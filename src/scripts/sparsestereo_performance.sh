#!/usr/bin/gawk -f

BEGIN {
	count = 0
}

{
	if($4 == "Fps:") {
		fps += $5;
		left += $10;
		right += $15;
		matches += gensub(/\%/, "", "", $20);
		count++;
	}
	
}

END {
	if(!quiet)
		printf "Avg. Fps: %f ; Avg. Left: %f ; Avg. Right: %f ; Avg. Match: %f\n",
			fps/count, left/count, right/count,  matches/count;
	else printf "%f %f %f %f\n", fps/count, left/count, right/count,  matches/count;
}
