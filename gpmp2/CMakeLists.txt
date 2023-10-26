# We split the library in to separate subfolders, each containing
# tests, timing, and an optional convenience library.
# The following variable is the master list of subdirs to add   
# 所有的子目录
set(gpmp2_subdirs   
    geometry
    gp 
    kinematics
    dynamics
    obstacle
    planner
    utils
)
set(gpmp2_srcs)
# set(gpmp2_srcs)创建了一个名为gpmp2_srcs的变量，并将其设置为空。这意味着在这个代码片段之前，gpmp2_srcs变量可能已经存在并包含了一些值，但通过执行这个set命令，它被清空或重置为一个空值。
# 在后续的代码中，gpmp2_srcs变量可能会通过list(APPEND gpmp2_srcs ...)命令添加新的值，或通过其他方式重新赋值。


# files want to be excluded
set(excluded_sources "")

# Library sources
foreach(subdir ${gpmp2_subdirs})
  file(GLOB subdir_srcs "${subdir}/*.cpp" "${subdir}/*.h")

  # 在代码的后续部分，可能会使用list(REMOVE_ITEM subdir_srcs "${excluded_sources}")命令从subdir_srcs列表中移除excluded_sources中指定的源文件。这样可以在构建过程中排除特定的源文件，而不将它们包含在最终的构建中。
  list(REMOVE_ITEM subdir_srcs "${excluded_sources}")

  file(GLOB subdir_test_files "${subdir}/tests/*")
  
  list(APPEND gpmp2_srcs ${subdir_srcs})
  message(STATUS "Building Module: ${subdir}")

  # local and tests
  add_subdirectory(${subdir})
endforeach(subdir)


# Versions
set(gpmp2_version   ${GPMP2_VERSION_MAJOR}.${GPMP2_VERSION_MINOR}.${GPMP2_VERSION_PATCH})
set(gpmp2_soversion ${GPMP2_VERSION_MAJOR})
message(STATUS "GPMP2 Version: ${gpmp2_version}")
message(STATUS "Install prefix: ${CMAKE_INSTALL_PREFIX}")


# 根据构建选项，通过条件判断选择构建静态库或共享库。
# build shared lib
if(GPMP2_BUILD_STATIC_LIBRARY)
  # static
  message(STATUS "Build static library")
  # 使用add_library()命令创建库，并使用target_link_libraries()命令链接所需的依赖库。
  add_library(${PROJECT_NAME} STATIC ${gpmp2_srcs})
  target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${GTSAM_LIBRARIES})
  set_target_properties(gpmp2 PROPERTIES
      OUTPUT_NAME         gpmp2
      CLEAN_DIRECT_OUTPUT 1
      VERSION             ${gpmp2_version}
      SOVERSION           ${gpmp2_soversion})
  if(WIN32) # Add 'lib' prefix to static library to avoid filename collision with shared library
    set_target_properties(gpmp2 PROPERTIES
      PREFIX "lib")
  endif()
  install(TARGETS ${PROJECT_NAME} EXPORT gpmp2-exports ARCHIVE DESTINATION lib)

else()
  # shared
  message(STATUS "Build shared library")
  add_library(${PROJECT_NAME} SHARED ${gpmp2_srcs})
  target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${GTSAM_LIBRARIES})
  set_target_properties(gpmp2 PROPERTIES
      OUTPUT_NAME         gpmp2
      CLEAN_DIRECT_OUTPUT 1
      VERSION             ${gpmp2_version}
      SOVERSION           ${gpmp2_soversion})
  if(WIN32)
    set_target_properties(gpmp2 PROPERTIES
        PREFIX ""
        DEFINE_SYMBOL GPMP2_EXPORTS
        RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bin")
  endif()
  install(TARGETS ${PROJECT_NAME} EXPORT gpmp2-exports LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)

endif()

list(APPEND GPMP2_EXPORTED_TARGETS gpmp2)
set(GPMP2_EXPORTED_TARGETS "${GPMP2_EXPORTED_TARGETS}" PARENT_SCOPE)

