if(NOT ENABLE_LINUX)
    find_package(SoapySDR REQUIRED)

    SOAPY_SDR_MODULE_UTIL(
            TARGET Microphase_IQTAXI
            SOURCES
            Registration.cpp
            iqtaxi_Streaming.cpp
            iqtaxi_Settings.cpp
            LIBRARIES
            ${SOAPYSDR_LIBRARIES}
            sdr_core
            sdr_driver
    )
endif()
