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
    # This should give you a valid catkin groovy workspace at ~/groovy_overlay

    # Next, check out a ROS stack that hasn't yet been converted to Catkin
    source ~/groovy_overlay/build/buildspace/setup.sh
    cd ~/groovy_overlay/src
    hg clone https://kforge.ros.org/common/filters
    cd filters

    # Convert CMakeLists.txt
    mv CMakeLists.txt CMakeLists.old
    catkinize_cmakelists.py filters CMakeLists.old manifest.xml > CMakeLists.txt
    # now check and adapt the CMakeLists.txt with any text editor
    $EDITOR CMakeLists.txt  # Make any changes needed

    # Convert manifest.xml to package.xml
    catkinize_manifest_xml_to_package_xml.py manifest.xml filters 1.6.0 \
      > package.xml

    # Edit package.xml:
    #  - Make sure there is a valid maintainer
    #  - Uncomment dependencies as needed
    $EDITOR package.xml

    # Check the results
    cd ~/groovy_overlay/build
    cmake ../src
    make

Background
----------

[This wiki page](http://www.ros.org/doc/groovy/api/catkin/html/user_guide/rosbuild_migration.html)
gives some instructions on how to catkinize ROS stacks.
More general information on writing CMakeLists.txt files using Catkin can
be found
[here](http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html).

The specification for package.xml files is in [REP
127](http://www.ros.org/reps/rep-0127.html).

