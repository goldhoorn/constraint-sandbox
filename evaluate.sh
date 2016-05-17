PLANNERS="BFSF CEDALION FDAUTOTUNE1 FDAUTOTUNE2 FDSS1 FDSS2 LAMA LAMA2011 RANDWARD UNIFORM"
#FILES="problem_orginal_depth_no_ports.pddl  problem_orginal_depth.pddl  problem_orginal_xsens_min.pddl  problem_orginal_xsens.pddl  problem_start.pddl  problem_stop.pddl problem_orginal_depth_sol2.pddl"
FILES="problem_start.pddl"

for file in $FILES;do 
	echo  $file >> result-$file.txt
	for i in $PLANNERS; do
		echo  "################  $file - $i ################################" >> result-$file.txt 
		/usr/bin/time pddl_planner_bin -l 1 $i -t 2000 -s domain.pddl diss-esamples/$file 2>&1 | tee -a result-$file.txt 
		echo  "################  Ende  ################################" >> result-$file.txt 
	done
done
