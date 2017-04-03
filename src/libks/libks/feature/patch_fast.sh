#!/usr/bin/gawk -f

/fast9CornerTest/ {
	greater = 0
	pixel = 0
	started = 1
	stackDepth = 0
}

/return false/ {
	print gensub(/false/, "0", "")
	next
}

/pixel/ {
	if(started) {
		split($0, parts, /[\[\]]/)
		pixel=parts[3]
		if(match($0, />/))
			greater=1;
		else
			greater=0;
		
		if(match($0, "else"))
			stackDepth--;
		
		stack[stackDepth, 0] = greater;
		stack[stackDepth, 1] = pixel;
		stackDepth++;
	}
	
	print gensub(/else/, "} else", "", gensub(/\)/, ") {", ""))
	next
}


/else/ {
	stackDepth--;
	
	print gensub(/else/, "} else", "")
	next
}

/{}/ {
	archLen=0
	# Extract the arch
	for(i in arch)
		delete arch[i]
	
	for(i=stackDepth-1; i>=0; i--) {
		if(stack[i, 0] == greater) {
			archLen++;
			arch[archLen] = stack[i, 1]
		}
	}
			
	# Sort and find the last pixel again
	n=asort(arch)
	
	startIndex=0
	for(i=1; i<=archLen; i++)
		if(arch[i] == pixel) {
			startIndex = i;
			break;
		}
	
	# Find start point
	lastPixel = arch[startIndex]
	for(i=startIndex-1;;i--) {
		if(i==0)
			i = archLen
		d = arch[i] - lastPixel
		if(d != 1 && d != -1 && d != 15 && d != -15) {
			archStart = lastPixel
			break;
		}
		lastPixel = arch[i]
	}
	
	# Find end point
	lastPixel = arch[startIndex]
	for(i=startIndex+1;;i++) {
		if(i>archLen)
			i = 1
		d = arch[i] - lastPixel
		if(d != 1 && d != -1 && d != 15 && d != -15) {
			archEnd = lastPixel
			break;
		}
		lastPixel = arch[i]
	}
	
	# Print return statement
	print gensub(/{}/, sprintf("return 0x%x%x%x; //pixel %d - %d", greater, archStart, archEnd, archStart, archEnd), $0)
	
	next
}

{
	print $0
}
