#idf_component_register(SRCS "timer-test.cpp"
#                    INCLUDE_DIRS ".")
cmake_minimum_required(VERSION 3.5)
if (NOT DEFINED PROJECT_NAME)
  project(timer-test)
endif (NOT DEFINED PROJECT_NAME)
idf_component_register(SRCS "timer-test.cpp"
                    INCLUDE_DIRS ".")
if (CONFIG_COMPILER_OPTIMIZATION_PERF)
    add_compile_definitions(RELEASE)
else()
    add_compile_definitions(DEBUG)
endif(CONFIG_COMPILER_OPTIMIZATION_PERF)