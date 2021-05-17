num_trials=10
folder="$(pwd)/mapping_$(date +%h%d-%H%M)_$num_trials"
hilo_path="/home/mpantic/ws/rmp/src/manifold_simulations/models/hilo_roof/meshes/hilo_reconstructed.off"
rhone_path="/home/mpantic/ws/rmp/src/manifold_simulations/models/rhone_mesh/meshes/rhone_enu_0.1m.off"
curve_path="/home/mpantic/Work/RAL_Manifolds/curve.off"

mkdir $folder
cd $folder
rosrun cpt_ompl_planning mapping_eval_node $num_trials $hilo_path > "hilo.log"
rosrun cpt_ompl_planning mapping_eval_node $num_trials $rhone_path > "rhone.log"
rosrun cpt_ompl_planning mapping_eval_node $num_trials $curve_path > "curve.log"

# filter data
cat "hilo.log" | grep "DATA" > "hilo_data.log"
cat "rhone.log" | grep "DATA" > "rhone_data.log"
cat "curve.log" | grep "DATA" > "curve_data.log"

# plot
#python3 ../plot.py
#cd ..