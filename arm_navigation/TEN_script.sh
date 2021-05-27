#!/bin/bash

# copy first letter to where parser wants it
cp config/desired_structures/{t,scripted}.txt
# run parser
python src/core.py
# run navigation
#move rail to next position
# copy second letter to where the parser wants it
# run parser
# run navigation
#move rail to last position
# copy last letter to where the parser expects it
# run parser
# run navigation
#move rail to home position
