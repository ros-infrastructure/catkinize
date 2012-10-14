Catkinize
=========

This is a collection of scripts to help convert ROS stacks to Catkin.

Installing
----------

	sudo python setup.py install	

Example
-------

    # Start by running through the getting started guide at
    # http://ros.org/doc/groovy/api/catkin/html/user_guide/getting_started.html

    # Next, check out a ROS stack that hasn't yet been converted to Catkin
    source ~/groovy_overlay/build/buildspace/setup.sh
    cd ~/groovy_overlay/src
	hg clone https://kforge.ros.org/common/filters
	cd filters

	# Set up a git repository
	git init
	git add .
	alias gcam='git commit -a -m'
	gcam 'Version before converting to Catkin'

	# Convert CMakeLists.txt
	mv CMakeLists.txt CMakeLists.old
	catkinize_cmakelists.py filters CMakeLists.old > CMakeLists.txt
	gcam 'Run catkinize_cmakelists'
	$EDITOR CMakeLists.txt  # Make any changes needed
	gcam 'More Catkinization of CMakeLists'

	# Convert manifest.xml to package.xml
	catkinize_manifest_xml_to_package_xml.py -a manifest.xml filters 1.6.0 \
		> package.xml
	git add package.xml
	gcam 'Generate package.xml from manifest.xml'
	# Edit package.xml:
	#  - Make sure there is a valid maintainer
	#  - Uncomment dependencies as needed
	$EDITOR package.xml
	gcam 'More Catkinization of package.xml'

    # Check the results
    cd ~/groovy_overlay/build
    cmake ../src
    make

Background
----------

[This wiki page](http://www.ros.org/doc/groovy/api/catkin/html/user_guide/rosbuild_migration.html)
gives some instruction on how to catkinize ROS stacks.
More general information on writing CMakeLists.txt files using Catkin can
be found
[here](http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html).

The specification for package.xml files is in [REP
127](http://www.ros.org/reps/rep-0127.html).

