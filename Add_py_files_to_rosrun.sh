#!/bin/bash
echo "This script will recursively apply write permissions to all *.py files in the ottershaw_masta catkin package." 
echo "Proceed? (y/n)"
read response

if [ "$response" == "y" ]; then
	cd ottershaw_masta
	sudo chmod -R +x */*.py
	echo "Applied execute permissions to all Python files in the ottershaw_masta package"
else
	echo "Operation aborted"
fi

