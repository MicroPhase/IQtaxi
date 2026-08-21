if(ENABLE_LIBUHD AND NOT ENABLE_WIN)
    #LIB UHD
    LIBUHD_REGISTER_COMPONENT("IQTAXI" ENABLE_IQTAXI_UHD ON "ENABLE_LIBUHD" OFF OFF)

    IF(ENABLE_IQTAXI_UHD)
        list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

        find_package(IQ_TAXI REQUIRED)

        message(STATUS "  Reading ${UHD_INCLUDE_DIRS}/uhd/device.hpp...")
        file(READ ${UHD_INCLUDE_DIRS}/uhd/device.hpp device_hpp)
        string(FIND "${device_hpp}" "device_filter_t" has_device_filter)
        if ("${has_device_filter}" STREQUAL "-1")
            message(STATUS "  has original API")
        else()
            target_compile_definitions(uhd PRIVATE UHD_HAS_DEVICE_FILTER)
            message(STATUS "  has filter API")
        endif()

        message(STATUS "Checking uhd::property::set_publisher() API...")
        message(STATUS "  Reading ${UHD_INCLUDE_DIRS}/uhd/property_tree.hpp...")
        file(READ ${UHD_INCLUDE_DIRS}/uhd/property_tree.hpp property_tree_hpp)
        string(FIND "${property_tree_hpp}" "set_publisher" has_set_publisher)
        if ("${has_set_publisher}" STREQUAL "-1")
            message(STATUS "  missing set_publisher() API")
        else()
            target_compile_definitions(uhd PRIVATE
                publish=set_publisher
                subscribe=add_desired_subscriber
                coerce=add_coerced_subscriber
            )
            message(STATUS "  has set_publisher() API")
        endif()

        LIBUHD_APPEND_SOURCES(
            ${CMAKE_CURRENT_SOURCE_DIR}/iqtaxi_impl.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/iqtaxi_io_impl.cpp
        )

        target_include_directories(uhd PRIVATE ${IQ_TAXI_INCLUDE_DIRS})
        LIBUHD_APPEND_LIBS(${IQ_TAXI_LIBRARIES})
    ENDIF(ENABLE_IQTAXI_UHD)
elseif(NOT ENABLE_WIN)
    #UHD Module Mode
    project(IQTaxiUHD CXX C)
    enable_testing()

    add_library(IQTaxiUHD SHARED iqtaxi_impl.cpp iqtaxi_io_impl.cpp)

    list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

    find_package(UHD REQUIRED)

    if(NOT UHD_ROOT)
        get_filename_component(UHD_ROOT "${UHD_INCLUDE_DIRS}/.." ABSOLUTE)
    endif()

    message(STATUS "UHD root directory: ${UHD_ROOT}")
    message(STATUS "UHD include directories: ${UHD_INCLUDE_DIRS}")
    message(STATUS "UHD libraries: ${UHD_LIBRARIES}")

    target_include_directories(IQTaxiUHD PRIVATE 
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${UHD_INCLUDE_DIRS}
    )

    message(STATUS "Checking uhd::device::register_device() API...")
    message(STATUS "  Reading ${UHD_INCLUDE_DIRS}/uhd/device.hpp...")
    file(READ ${UHD_INCLUDE_DIRS}/uhd/device.hpp device_hpp)
    string(FIND "${device_hpp}" "device_filter_t" has_device_filter)
    if ("${has_device_filter}" STREQUAL "-1")
        message(STATUS "  has original API")
    else()
        target_compile_definitions(IQTaxiUHD PRIVATE UHD_HAS_DEVICE_FILTER)
        message(STATUS "  has filter API")
    endif()

    message(STATUS "Checking uhd::property::set_publisher() API...")
    message(STATUS "  Reading ${UHD_INCLUDE_DIRS}/uhd/property_tree.hpp...")
    file(READ ${UHD_INCLUDE_DIRS}/uhd/property_tree.hpp property_tree_hpp)
    string(FIND "${property_tree_hpp}" "set_publisher" has_set_publisher)
    if ("${has_set_publisher}" STREQUAL "-1")
        message(STATUS "  missing set_publisher() API")
    else()
        target_compile_definitions(IQTaxiUHD PRIVATE
            publish=set_publisher
            subscribe=add_desired_subscriber
            coerce=add_coerced_subscriber
        )

        message(STATUS "  has set_publisher() API")
    endif()


    if(TARGET sdr_driver)
        message(STATUS "Using in-tree IQTAXI driver library for UHD module")
        set(IQTAXI_UHD_DRIVER_LIBS sdr_driver)
        target_include_directories(IQTaxiUHD PRIVATE ${SDR_INCLUDE_DIR})
    else()
        find_package(IQ_TAXI REQUIRED)

        target_include_directories(IQTaxiUHD PRIVATE ${IQ_TAXI_INCLUDE_DIRS})

        message(STATUS "IQ Taxi include directories: ${IQ_TAXI_INCLUDE_DIRS}")
        message(STATUS "IQ Taxi libraries: ${IQ_TAXI_LIBRARIES}")
        set(IQTAXI_UHD_DRIVER_LIBS ${IQ_TAXI_LIBRARIES})
    endif()

    set(BOOST_REQUIRED_COMPONENTS
        thread
        chrono
        atomic
        system
    )
    
    find_package(Boost COMPONENTS ${BOOST_REQUIRED_COMPONENTS})

    if(NOT Boost_FOUND)
        message(WARNING "Boost not found -- required for IQTaxiUHD support")
        return()
    endif()

    target_include_directories(IQTaxiUHD PRIVATE ${Boost_INCLUDE_DIRS})
    target_link_directories(IQTaxiUHD PRIVATE ${Boost_LIBRARY_DIRS})


    message(STATUS "Boost include directories: ${Boost_INCLUDE_DIRS}")
    message(STATUS "Boost library directories: ${Boost_LIBRARY_DIRS}")
    message(STATUS "Boost libraries: ${Boost_LIBRARIES}")

    target_link_libraries(IQTaxiUHD
        ${UHD_LIBRARIES}
        ${Boost_LIBRARIES}
        ${IQTAXI_UHD_DRIVER_LIBS}
    )

    install(TARGETS IQTaxiUHD
        DESTINATION ${UHD_ROOT}/lib/uhd/modules
    )

endif()
