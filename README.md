Catkinize
=========

This is a collection of scripts to convert ROS stacks to Catkin.


Installing
----------

	sudo python setup.py install	

Example
-------

	hg clone https://kforge.ros.org/common/filters
	cd filters
	git init
	git add .
	alias gcam='git commit -a -m'
	gcam 'Version before converting to Catkin'
	catkinize_cmakelists.py filters CMakeLists.txt	
	gcam 'Run catkinize_cmakelists'
	$EDITOR CMakeLists.txt  # Make any changes needed
	gcam 'More Catkinization of CMakeLists'
	catkinize_manifest_xml_to_package_xml.py -a manifest.xml filters 1.6.0 \
		> package.xml
	git add package.xml
	gcam 'Generate package.xml from manifest.xml'
	$EDITOR package.xml   # Make any changes needed
	gcam 'More Catkinization of package.xml'

Background
----------

[This wiki page](http://www.ros.org/doc/groovy/api/catkin/html/user_guide/rosbuild_migration.html)
gives some instruction on how to catkinize.
More general information on writing CMakeLists.txt files using Catkin can
be found
[here](http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html)

The specification for package.xml files is in [REP
127](http://www.ros.org/reps/rep-0127.html).

