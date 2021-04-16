#!/usr/bin/env python3

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import errno
import argparse
import os
import sys
import re

import rospy

import std_msgs.msg

import baxter_dataflow

from baxter_maintenance_msgs.msg import (
    UpdateSources,
    UpdateStatus,
)


class Updater(object):
    """
    Control the updater on the robot.

    Signals:
        status_changed:     Fired when the update status changes.  Passes
                            the current UpdateStatus message.
    """
    def __init__(self):
        self.status_changed = baxter_dataflow.Signal()

        self._status = UpdateStatus()
        self._avail_updates = UpdateSources()

        self._update_sources = rospy.Subscriber(
            '/usb/update_sources',
            UpdateSources,
            self._on_update_sources)

        self._updater_status_sub = rospy.Subscriber(
            '/updater/status',
            UpdateStatus,
            self._on_update_status)

        self._updater_start = rospy.Publisher(
            '/updater/start',
            std_msgs.msg.String,
            queue_size=10)

        self._updater_stop = rospy.Publisher(
            '/updater/stop',
            std_msgs.msg.Empty,
            queue_size=10)

        baxter_dataflow.wait_for(
            lambda: self._avail_updates.uuid != '',
            timeout=5.0,
            timeout_msg="Failed to get list of available updates"
        )

    def _on_update_sources(self, msg):
        if msg.uuid != self._avail_updates.uuid:
            self._avail_updates = msg

    def _on_update_status(self, msg):
        if self._status != msg:
            self._status = msg
            self.status_changed(msg)

    def list(self):
        """
        Return a list of tuples (version, uuid) of all available updates
        """
        return [(u.version, u.uuid) for u in self._avail_updates.sources]

    def command_update(self, uuid):
        """
        Command the robot to launch the update with the given uuid.

        @param uuid: uuid of the update to start.
        """
        if not any([u.uuid == uuid for u in self._avail_updates.sources]):
            raise OSError(errno.EINVAL, "Invalid update uuid '%s'" % (uuid,))

        self._updater_start.publish(std_msgs.msg.String(uuid))

    def stop_update(self):
        """
        Stop the current update process, if any.
        """
        self._updater_stop.publish()


def run_update(updater, uuid):
    """
    Run and monitor the progress of an update.

    @param updater: Instance of Updater to use.
    @param uuid: update uuid.
    """

    # Work around lack of a nonlocal keyword in python 2.x
    class NonLocal(object):
        pass

    nl = NonLocal
    nl.rc = 1
    nl.done = False

    def on_update_status(msg):
        if msg.status == UpdateStatus.STS_IDLE:
            nl.done = True
        elif msg.status == UpdateStatus.STS_INVALID:
            print ("Invalid update uuid, '%s'." % (uuid,))
            nl.done = True
        elif msg.status == UpdateStatus.STS_BUSY:
            print ("Update already in progress (may be shutting down).")
            nl.done = True
        elif msg.status == UpdateStatus.STS_CANCELLED:
            print ("Update cancelled.")
            nl.done = True
        elif msg.status == UpdateStatus.STS_ERR:
            print ("Update failed: %s." % (msg.long_description,))
            nl.done = True
            nl.rc = 1
        elif msg.status == UpdateStatus.STS_LOAD_KEXEC:
            print ("Robot will now reboot to finish updating...")
            nl.rc = 0
        else:
            print ("Updater:  %s" % (msg.long_description))

    def on_shutdown():
        updater.stop_update()
    rospy.on_shutdown(on_shutdown)

    updater.status_changed.connect(on_update_status)

    try:
        updater.command_update(uuid)
    except OSError as e:
        if e.errno == errno.EINVAL:
            print(e.strerror)
            return 1
        raise

    try:
        baxter_dataflow.wait_for(
            lambda: nl.done == True,
            timeout=5 * 60,
            timeout_msg="Timeout waiting for update to succeed"
        )
    except Exception as e:
        if not (hasattr(e, 'errno') and e.errno == errno.ESHUTDOWN):
            print(e.strerror)
        nl.rc = 1

    return nl.rc


def ros_updateable_version():
    """
    Verifies the version of the software is older than 1.1.0.
    If newer, print out url to SSH update instructions
    and return False
    """
    ros_updateable = True

    def get_robot_version():
        param_name = "rethink/software_version"
        robot_version = rospy.get_param(param_name, None)
        # parse out first 3 digits of robot version tag
        pattern = ("^([0-9]+)\.([0-9]+)\.([0-9]+)")
        match = re.search(pattern, robot_version)
        if not match:
            rospy.logwarn("RobotUpdater: Invalid robot version: %s",
                          robot_version)
            return None
        return match.string[match.start(1):match.end(3)]

    robot_version = get_robot_version()
    if not robot_version:
        rospy.logerr("RobotUpdater: Failed to retrieve robot version "
                     "from rosparam: %s\n"
                     "Verify robot state and connectivity "
                     "(i.e. ROS_MASTER_URI)", param_name)
        ros_updateable = False
    else:
        v_list = robot_version.split('.')
        # If Version >= 1.1, SSH required
        if ((float(v_list[0]) > 1) or
           (float(v_list[0]) == 1 and float(v_list[1]) >= 1)):
            errstr_version = ("RobotUpdater: Your Baxter's software "
                              "version is (%s).\nFor robots with software "
                              "1.1.0 and newer, software updates must be run "
                              "by SSH'ing into the robot\n"
                              "or by switching to Baxter's Display TTY1 with "
                              "a keyboard. Be sure to insert a USB flash "
                              "drive\ncontaining the update. "
                              "After logging into the robot, run:\n"
                              "  ruser@baxter ~ $ rethink-updater --list\n"
                              "  Version\n"
                              "  some.version.num.ber\n"
                              "  ruser@baxter ~ $ rethink-updater --update "
                              "some.version.num.ber\n"
                              "Please see "
                              "http://sdk.rethinkrobotics.com/wiki/SSH_Update "
                              "for a detailed tutorial on updating.")
            rospy.logerr(errstr_version, robot_version)
            ros_updateable = False
    return ros_updateable


def main():
    description = "Legacy Robot Updater Script for versions 1.0.0 and earlier."
    parser = argparse.ArgumentParser(description=description)
    required = parser.add_mutually_exclusive_group()
    required.add_argument('-l', '--list', action='store_const',
                          dest='cmd', const='list', default='update',
                          help="List available updates and UUID's")
    required.add_argument('-u', '--update', dest='uuid', default='',
                          help='Launch the given update')
    args = parser.parse_args(rospy.myargv()[1:])
    cmd = args.cmd
    uuid = args.uuid

    rospy.init_node('rsdk_update_robot')
    if not ros_updateable_version():
        sys.exit(1)
    updater = Updater()

    if cmd == 'list':
        updates = updater.list()
        if not len(updates):
            print ("No available updates")
        else:
            print ("%-30s%s" % ("Version", "UUID"))
            for update in updates:
                print("%-30s%s" % (update[0], update[1]))
        return 0
    elif cmd == 'update':
        if uuid == '':
            print("Error:  no update uuid specified")
            return 1
        msg = ("NOTE: Please plug in any Rethink Electric Parallel Grippers\n"
               "      into the robot now, so that the Gripper Firmware\n"
               "      can be automatically upgraded with the robot.\n")
        print (msg)
        raw_input("Press <Enter> to Continue...")
        if rospy.is_shutdown():
            return 0
        return run_update(updater, uuid)

if __name__ == '__main__':
    sys.exit(main())
