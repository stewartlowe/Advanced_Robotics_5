function results(sensor_noise,movement_noise,num_sims)

	%initialise number of successes/kinda successes at 0
	successes = 0;
	kinda_successes = 0;

	%loop for the required number of simulations
	for i=1:num_sims

		%Store outputs from trinity02 function in array
		%   Cycles      |  Travel Dist  |     Goal      |   Obs. Crash  |   Wall Crash  |
		[result_tab(i,1),result_tab(i,2),result_tab(i,3),result_tab(i,4),result_tab(i,5)] = trinity02(sensor_noise,movement_noise);
		
		%If goal is reached...
		if result_tab(i,3)==1
			%...with no collisions, add 1 to successes
			if result_tab(i,4)==0
				successes = successes+1;
			%...with collisions, add 1 to kinda successes
			else
				kinda_successes = kinda_successes+1;
			end
		end
	end
	
	%Calculate and display success rate
	success_rate = successes/num_sims
	kinda_success_rate = kinda_successes/num_sims
	overall_rate = success_rate + kinda_success_rate
	%calculate the average number of computing cycles for the sucessful sims
	ind = find(result_tab(:,3)==1);
	avg_cyc_no_crashes = mean(result_tab(ind,1))
	ind = find(result_tab(:,3)==1 & result_tab(:,4) > 0);
	avg_cyc_with_crashes = mean(result_tab(ind,1))
	avg_cyc_overall = (avg_cyc_with_crashes + avg_cyc_no_crashes)/2
	
end