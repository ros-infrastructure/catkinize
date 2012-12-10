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
    catkinize_stack filters

    # now check and adapt the CMakeLists.txt with any text editor
    $EDITOR CMakeLists.txt  # Make any changes needed
    # Edit package.xml:
    #  - Make sure there is a valid maintainer
    #  - Uncomment dependencies as needed
    $EDITOR package.xml

    # Check the results
    cd ~/groovy_overlay/build
    cmake ../src
    make

    # delete the obsolete backup files once you don't need them anymore
    cd ~/groovy_overlay/src/filters
    # check those are all files you want to be gone
    find  . -name \*.backup
    # delete
    find . -name \*.backup -exec rm {} \;

Background
----------

[This wiki page](http://www.ros.org/doc/groovy/api/catkin/html/user_guide/rosbuild_migration.html)
gives some instructions on how to catkinize ROS stacks.
More general information on writing CMakeLists.txt files using Catkin can
be found
[here](http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html).

The specification for package.xml files is in [REP
127](http://www.ros.org/reps/rep-0127.html).

