cmake_minimum_required(VERSION 3.0.2)
project(hex_arm_controller)

find_package(catkin REQUIRED COMPONENTS
    rospy
    )


catkin_package()

foreach(dir config launch)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

file(GLOB SCRIPT_FILES scripts/*)
foreach(SCRIPT ${SCRIPT_FILES})
    if(IS_DIRECTORY ${SCRIPT})
        install(DIRECTORY ${SCRIPT}/
            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
        )
    elseif(${SCRIPT} MATCHES "__init__\\.py$")
        install(FILES ${SCRIPT}
            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
        )
    elseif(${SCRIPT} MATCHES "\\.py$")
        install(PROGRAMS ${SCRIPT}
            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
        )
    endif()
endforeach()
