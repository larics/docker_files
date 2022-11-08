DROP_VAL=0.2

docker start mbzirc_ecc22_cont
docker exec mbzirc_ecc22_cont bash -c "sed -i '71s/.*/      <drop_probability>${DROP_VAL}<\/drop_probability>/' /home/developer/mbzirc_ws/src/simulator-workspace/mbzirc_launch/worlds/simple_demo_manip_uav.sdf"
docker exec mbzirc_ecc22_cont bash -c "sed -i '34s/.*/        loss = ${DROP_VAL}/' /home/developer/mbzirc_ws/src/mbzirc_aerial_manipulation/scripts/monitor_topic.py"
docker exec mbzirc_ecc22_cont bash -c "cd /home/developer/mbzirc_ws/; colcon build --merge-install --packages-select=mbzirc_launch"
docker exec mbzirc_ecc22_cont bash -c "cd /home/developer/mbzirc_ws/; colcon build --merge-install --packages-select=mbzirc_aerial_manipulation"
docker stop mbzirc_ecc22_cont

sleep 5
n=2
for i in $(seq $n); do
    tmuxinator start mbzirc_ecc22
done

DROP_VAL=0.5

docker start mbzirc_ecc22_cont
docker exec mbzirc_ecc22_cont bash -c "sed -i '71s/.*/      <drop_probability>${DROP_VAL}<\/drop_probability>/' /home/developer/mbzirc_ws/src/simulator-workspace/mbzirc_launch/worlds/simple_demo_manip_uav.sdf"
docker exec mbzirc_ecc22_cont bash -c "sed -i '34s/.*/        loss = ${DROP_VAL}/' /home/developer/mbzirc_ws/src/mbzirc_aerial_manipulation/scripts/monitor_topic.py"
docker exec mbzirc_ecc22_cont bash -c "cd /home/developer/mbzirc_ws/; colcon build --merge-install --packages-select=mbzirc_launch"
docker exec mbzirc_ecc22_cont bash -c "cd /home/developer/mbzirc_ws/; colcon build --merge-install --packages-select=mbzirc_aerial_manipulation"
docker stop mbzirc_ecc22_cont

sleep 5
n=2
for i in $(seq $n); do
    tmuxinator start mbzirc_ecc22
done




