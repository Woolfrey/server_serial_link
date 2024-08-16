#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Mar 16 09:09:39 2024

@author: woolfrey

User survey conducted the week of March 10th 2024
n = 8

"""

import matplotlib.pyplot as plt
import numpy as np


############################################################
#   What kind of actions do you perform with a robot arm?  #
############################################################

actions = ("Moving to a pre-defined joint position \n (the trajectory is solved for me)",
           "Moving through multiple pre-defined joint positions \n (the trajectory is solved for me)",
           "Tracking a pre-defined joint-space trajectory \n (I define the trajectory)",
           "Moving to a specified endpoint transform \n (in a straight line)",
           "Moving through multiple pre-defined endpoint transforms \n (the trajectory is solved for me)",
           "Tracking a pre-defined joint trajectory \n (I define the trajectory)",
           "Moving the endpoint at a given speed \n (e.g. manual input)",
           "Following a Cartesian transform in real time",
           "Moving indirectly to a specified endpoint transform \n (I don't care where it moves inbetween)")
           
action_votes = [6,4,4,4,3,3,3,2,1]

fig1, ax1 = plt.subplots(figsize=(15,6))

fig1.set_figwidth(5)

ax1.barh(np.arange(len(actions)),
        action_votes,
        align = 'center',
        height=0.8)

ax1.set_yticks(np.arange(len(actions)),
              labels=actions)

ax1.set_xlabel("Votes")
ax1.invert_yaxis()
ax1.set_title("What kind of actions do you perform with the robot?")
ax1.spines[['right', 'top']].set_visible(False)

############################################################
#   What kind of information are you interested in         #
#   while the robot is running?                            #
############################################################

feedback = ("Position tracking error",
            "Time to completion",
            "Proximity to a singularity",
            "Velocity tracking error")

feedback_votes = [7,5,4,2]

fig2, ax2 = plt.subplots()

ax2.barh(np.arange(len(feedback)),
         feedback_votes,
         align = 'center')

ax2.set_yticks(np.arange(len(feedback)),
              labels=feedback)

ax2.set_xlabel("Votes")
ax2.invert_yaxis()
ax2.set_title("What kind of information are you interested in \n while the robot is running?")
ax2.spines[['right', 'top']].set_visible(False)

############################################################
#       What kind of results are you interested in         # 
#       when the action is complete?                       #
############################################################

result = ("Successful or not \n (it failed for some reason)",
          "Mean position error, variance",
          "Average speed, variance")

result_votes = [8,7,3]

fig3, ax3 = plt.subplots()

ax3.barh(np.arange(len(result)),
         result_votes,
         align = 'center')

ax3.set_yticks(np.arange(len(result)),
              labels=result)

ax3.set_xlabel("Votes")
ax3.invert_yaxis()
ax3.set_title("What kind of information are you interested in \n when the action is complete?")
ax3.spines[['right', 'top']].set_visible(False)

plt.show()