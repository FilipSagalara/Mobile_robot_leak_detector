#! /usr/bin/env python3
#! /usr/bin/env bash

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from rotate_angle import Spin


class TriggerService:
    def __init__(self, service_name):
        rospy.init_node("service_handler")
        if service_name[0] != "/":
            service_name = "/" + str(service_name)
        self.my_service = rospy.Service(
            str(service_name), Trigger, self.trigger_response
        )
        self.spinner = Spin()
        print("Service initialized waiting for incoming calls")
        self.function_to_execute = None
        rospy.spin()

    def trigger_response(self, request):
        self.spinner.spin(30)

        return TriggerResponse(
            success=True, message="Hey, roger that; we'll be right there!"
        )


if __name__ == "__main__":

    s1 = TriggerService("test")
