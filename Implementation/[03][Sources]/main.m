%╭─╮╭─╮╭─╮╱╱╱╱╱╱╱╱╱╭╮╱╭┬─╮╭─╮╭─╮┬─╮┌─╮╭─╮╱╱╱╱╭─╮┬─╮┌─╱╱╱╱╱╱╱╱╱┌─╮╭─╮╱╱╱╱╭─╮╭─╮╭─╮┬─╮┌─╮╭─╮╱╱╱╱╱╱╱╱╱╭┬─╮┬─╮
%|├─╯└─╯├─╯╱╱╱╱╱╱╱╱╱┃┃╱├┴┐├─╯├─╯├─┤└─╯├─╯╱╱╱╱├─╯├─┤└─╱╱╱╱╱╱╱╱╱└─╯├─╯╱╱╱╱├─╯├─┤└─╯├─╯╱╱╱╱╱╱╱╱├─╯├─┤└─╱╱   |
%|┴─╮┌─╮┴─╮╭┬─╮╱╱╱╱╱╱┃┃╱├─┤└─╯├─╮┴─┤┌─╮┴─╮╭┬─╮├─╮┴─┤┌─╱╱╱╱╱╱╱╱╱┌─╮┴─╮╭┬─╮├─╮┴─╮╭┬─╮╱╱╱╱╱╱╱╱├─╮┴─╮╭┬─╮├─╮ |
%|─╯└─╯┴─╯╰┴─╯╰─────────╯╱╰─╯┴──╯╰─╯─╯┴─╯╰┴─╯╰──╯╰─╯─╯──────────╯┴─╯╰┴─╯╰─╯─╯┴─╯╰┴─╯╰─────────╯╱╰─╯┴──╯╰─|
%|                                                                                                       |
%|                                      Variable Truss Topology Robot                                    |
%|                                         MATLAB Implementation                                         |
%|                                                                                                       |
%|───╮┌─╮╭┬─╮╱╱╱╱╱╱┃┃╱├─┤└─╯├─╮┴─┤┌─╮┴─╮╭┬─╮├─╮┴─┤┌─╱╱╱╱╱╱╱╱╱┌─╮┴─╮╭┬─╮├─╮┴─╮╭┬─╮╱╱╱╱╱╱╱╱├─╮┴─╮╭┬─╮├─╮ ├─|
%|─╯└─╯╰┴─╯╰─────────╯╰─╰─╯┴──╯╰─╯─╯┴─╯╰┴─╯╰──╯╰─╯─╯──────────╯┴─╯╰┴─╯╰─╯─╯┴─╯╰┴─╯╰────────╯╰──╯┴┴┴┴──╯╰─|
%|                                                                                                       |
%|                  Author: Ozgur Gulsuna                                                                |
%|                  Date: 07-01-2024                                                                     |
%|                                                                                                       |
%|                  Description:                                                                         |
%|                  This MATLAB code implements a variable truss topology robot, allowing                |
%|                  for flexibility in the design of the truss structure.                                |
%|                                                                                                       |
%|                  Usage:                                                                               |
%|                  - Ensure you have the required toolboxes and dependencies installed.                 |
%|                  - Set up your robot parameters and desired truss topology.                           |
%|                  - Run the script to simulate and analyze the robot's performance.                    |
%|                                                                                                       |
%|                  Note:                                                                                |
%|                  Customize this header with relevant information for your application.                |
%|                                                                                                       |
%|                  Acknowledgements:                                                                    |
%|                  This code was developed as part of the MECH 6313 - Advanced Robotics course at       |
%|                  Carleton University.                                                                 |
%|                                                                                                       |
%|                                                                                                       |
%|                                                                                                       |
%|                                                                                                       |
%|╭─╮╭─╮╭─╮╱╱╱╱╱╱╱╱╱╭╮╱╭┬─╮╭─╮╭─╮┬─╮┌─╮╭─╮╱╱╱╱╭─╮┬─╮┌─╱╱╱╱╱╱╱╱╱┌─╮╭─╮╱╱╱╱╭─╮╭─╮╭─╮┬─╮┌─╮╭─╮╱╱╱╱╱╱╱╱╱╭─╮┬─|
%|─╯└─╯├─╯╱╱╱╱╱╱╱╱╱┃┃╱├┴┐├─╯├─╯├──╮┤└─╯├─╯╱╱╱╱├─╯├─┤└─╱╱╱╱╱╱╱╱╱└─╯├─╯╱╱╱╱├─╯├──╮┤└─╯├─╯╱╱╱╱╱╱╱╱├─╯├─┤└─╱╱|
%└─╯┴─╯╰┴─╯╰─────────╯╱╰─╯┴──╯╰─╯─╯┴─╯╰┴─╯╰──╯╰─╯─╯──────────╯┴─╯╰┴─╯╰─╯─╯┴─╯╰┴─╯╰─────────╯╱╰─╯┴┴┴┴──╯╰─╯
