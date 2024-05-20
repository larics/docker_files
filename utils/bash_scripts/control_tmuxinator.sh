OUTPUT="docker inspect -f '{{.State.Status}}' mbzirc_ecc22_cont"; 
while [ `$OUTPUT | grep -c running` = 1 ]; do 
  echo True;
  sleep 1;  
  OUTPUT=$OUTPUT; 
done

tmuxinator stop mbzirc_ecc22
