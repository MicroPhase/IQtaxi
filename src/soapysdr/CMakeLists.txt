if((ENABLE_E100 OR ENABLE_E200) AND NOT ENABLE_WIN)
    add_subdirectory(IQTAXI)
endif()
