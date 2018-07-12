#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, TriggerRequest
from art_msgs.srv import ProgramIdTrigger, ProgramIdTriggerResponse, ProgramIdTriggerRequest


class ControlProgramRun:

    def __init__(self):
        self.user_presence_sub = rospy.Subscriber('/art/interface/user/present', Bool,
                                                  self.user_presence_cb)
        self.pause_program_client = rospy.ServiceProxy("/art/brain/program/pause", Trigger)
        self.resume_program_client = rospy.ServiceProxy("/art/brain/program/resume", Trigger)
        self.stop_program_client = rospy.ServiceProxy("/art/brain/program/stop", Trigger)
        self.start_program_client = rospy.ServiceProxy("/art/brain/program/start", ProgramIdTrigger)
        self.time_user_appeared = None
        self.time_user_disappeared = None

        self.demo_program = rospy.get_param("~demo_program", 1)

        resp = self.start_program_client.call(ProgramIdTriggerRequest(program_id=self.demo_program))
        if not resp.success:
            rospy.logerr("Demo program could not be launched, exiting!")
            rospy.signal_shutdown("Demo program could not be launched")
        self.program_is_running = True
        self.program_is_paused = False

    def user_presence_cb(self, data):
        if data.data:  # user is presented
            self.time_user_disappeared = None
            if self.time_user_appeared is None:  # user just arrived
                self.time_user_appeared = rospy.Time.now()
                if not self.program_is_paused:
                    resp = self.pause_program_client.call(TriggerRequest())
                    if not resp.success:
                        rospy.logerr("Failed to pause program!")  # TODO: what to do?
                    self.program_is_paused = True
            elif self.program_is_running and (rospy.Time.now() - self.time_user_appeared) > rospy.Duration(2):
                # user is at least 2 seconds in front of the table -> stop program
                resp = self.stop_program_client.call(TriggerRequest())
                if not resp.success:
                    rospy.logerr("Failed to stop program!")  # TODO: what to do?
                self.program_is_running = False
                self.program_is_paused = False
        else:
            self.time_user_appeared = None
            if self.time_user_disappeared is None:
                self.time_user_disappeared = rospy.Time.now()
                if self.program_is_paused and self.program_is_running:
                    resp = self.resume_program_client.call(TriggerRequest())
                    if not resp.success:
                        rospy.logerr("Failed to resume program!")  # TODO: what to do?
                    self.program_is_paused = False
                elif not self.program_is_running:
                    resp = self.start_program_client.call(ProgramIdTriggerRequest(program_id=self.demo_program))
                    if not resp.success:
                        rospy.logerr("Demo program could not be launched!")
                    self.program_is_running = True
                    self.program_is_paused = False
            elif (rospy.Time.now() - self.time_user_disappeared) > rospy.Duration(2) and not self.program_is_running:
                resp = self.start_program_client.call(ProgramIdTriggerRequest(program_id=self.demo_program))
                if not resp.success:
                    rospy.logerr("Demo program could not be launched!")
                self.program_is_running = True
                self.program_is_paused = False


if __name__ == '__main__':
    rospy.init_node('control_program_run')
    ''',log_level=rospy.DEBUG'''

    try:
        node = ControlProgramRun()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
