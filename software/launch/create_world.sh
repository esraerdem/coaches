#!/bin/bash
../external/stage_environments/scripts/create_world.py Rive1 segway diago 10 18 0 ../external/stage_environments/maps/AUTOGEN_Rive1_diago.world

echo "# boxes" >> ../external/stage_environments/maps/AUTOGEN_Rive1_diago.world
echo "box( pose [ 15.0 19.0 0.1 0 ] color \"blue\" name \"personBlue\" )" >> ../external/stage_environments/maps/AUTOGEN_Rive1_diago.world
echo "box( pose [ 17.0 19.0 0 45 ] color \"red\" name \"personRed\")" >> ../external/stage_environments/maps/AUTOGEN_Rive1_diago.world
echo "box( pose [ 16.0 16.0 0.1 0 ] color \"pink\" name \"personPink\")" >> ../external/stage_environments/maps/AUTOGEN_Rive1_diago.world



../external/stage_environments/scripts/create_world.py DISB1 segway diago 2.6 13 180 ../external/stage_environments/maps/AUTOGEN_DISB1_diago.world

echo "# boxes" >> ../external/stage_environments/maps/AUTOGEN_DISB1_diago.world
echo "box( pose [ 2.0 25.0 0.1 0 ] color \"blue\" name \"personBlue\" )" >> ../external/stage_environments/maps/AUTOGEN_DISB1_diago.world
echo "box( pose [ 2.0 27.0 0 45 ] color \"red\" name \"personRed\")" >> ../external/stage_environments/maps/AUTOGEN_DISB1_diago.world
echo "box( pose [ 2.0 29.0 0.1 0 ] color \"pink\" name \"personPink\")" >> ../external/stage_environments/maps/AUTOGEN_DISB1_diago.world
echo "box( pose [ 3.3 21.0 0 0 ] color \"brown\" name \"printer\")" >> ../external/stage_environments/maps/AUTOGEN_DISB1_diago.world
