# A function 
function( GIT_VERSION VAR_PREFIX)

set( GIT_INFO ID AUTHOR DATE BRANCH DESC )

set( GIT_COMMIT_ID_DEFAULT      "UNKNOWN" )
set( GIT_COMMIT_AUTHOR_DEFAULT  "UNKNOWN" )
set( GIT_COMMIT_DATE_DEFAULT    "UNKNOWN" )
set( GIT_COMMIT_BRANCH_DEFAULT  "UNKNOWN" )
set( GIT_COMMIT_DESC_DEFAULT    "None" )

# we need git of course...
find_package( Git )
if( GIT_FOUND )
  # make sure that this is actually a git repo
  execute_process( COMMAND ${GIT_EXECUTABLE} status
                   WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                   RESULT_VARIABLE IsGitRepo
                   OUTPUT_VARIABLE OutputTrash
                   ERROR_VARIABLE ErrorTrash)
else()
  message( FATAL_ERROR "Could not find git command" )
endif()

if( ${IsGitRepo} EQUAL 0 )
    if( "${GIT_COMMIT_ID}" STREQUAL "" )
      execute_process( COMMAND ${GIT_EXECUTABLE} rev-parse --sq HEAD 
                       WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                       OUTPUT_VARIABLE GIT_COMMIT_ID
                       ERROR_VARIABLE Trash)
    endif()
    if( "${GIT_COMMIT_AUTHOR}" STREQUAL "" )
      execute_process( COMMAND ${GIT_EXECUTABLE} log -n1 --pretty="%an" HEAD
                       WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                       OUTPUT_VARIABLE GIT_COMMIT_AUTHOR
                       ERROR_VARIABLE Trash)
    endif()
    if( "${GIT_COMMIT_DATE}" STREQUAL "" )
      execute_process( COMMAND ${GIT_EXECUTABLE} log -n1 --pretty="%aD" HEAD
                       WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                       OUTPUT_VARIABLE GIT_COMMIT_DATE
                       ERROR_VARIABLE Trash)
    endif()
    if( "${GIT_COMMIT_BRANCH}" STREQUAL "" )
      execute_process( COMMAND ${GIT_EXECUTABLE} rev-parse --abbrev-ref HEAD
                       WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                       OUTPUT_VARIABLE GIT_COMMIT_BRANCH
                       ERROR_VARIABLE Trash)
    endif()
    if( "${GIT_COMMIT_DESC}" STREQUAL "" )
      execute_process( COMMAND ${GIT_EXECUTABLE} describe --tags HEAD
                       WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                       OUTPUT_VARIABLE GIT_COMMIT_DESC
                       ERROR_VARIABLE Trash)
    endif()
else()
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/version.txt")
    message( WARNING "Source directory is not a git repo, but 'version.txt' was found. Using version number there." )
    if( "${GIT_COMMIT_ID}" STREQUAL "" )
      set( GIT_COMMIT_ID "UNKNOWN" )
    endif()
    if( "${GIT_COMMIT_AUTHOR}" STREQUAL "" )
      set( GIT_COMMIT_AUTHOR "UNKNOWN" )
    endif()
    if( "${GIT_COMMIT_DATE}" STREQUAL "" )
      set( GIT_COMMIT_DATE "UNKNOWN" )
    endif()
    if( "${GIT_COMMIT_BRANCH}" STREQUAL "" )
      set( GIT_COMMIT_BRANCH "UNKNOWN" )
    endif()
    if( "${GIT_COMMIT_DESC}" STREQUAL "" )
      file( READ "${CMAKE_CURRENT_SOURCE_DIR}/version.txt" GIT_COMMIT_DESC )
    endif()
  else()
    message( FATAL_ERROR "Source directory is not a git repo." )
endif()
endif()

# set defaults for any items that were not found
foreach( item ${GIT_INFO} )
  if( "${GIT_COMMIT_${item}}" STREQUAL "" )
    set( GIT_COMMIT_${item} ${GIT_COMMIT_${item}_DEFAULT} )
  endif()
endforeach( item )

# remove quotes, backslashes, and new lines
# these cause problems when trying to embed version numbers into source files.
foreach( item ${GIT_INFO} )
  string( REPLACE "'"  "" GIT_COMMIT_${item}   ${GIT_COMMIT_${item}} )
  string( REPLACE "\"" "" GIT_COMMIT_${item}   ${GIT_COMMIT_${item}} )
  string( REPLACE "\n" "" GIT_COMMIT_${item}   ${GIT_COMMIT_${item}} )
endforeach( item )

# set full version string
set(VERSION_FULL "${GIT_COMMIT_DESC}-${GIT_COMMIT_BRANCH}" )

# look for major, minor, and patch numbers in the version string
# this is required if the developer uses CPack for example.
# need to consider two possible formats:
#    Major.Minor.Patch
# or 
#    Major.Minor-PatchStr
# the first format is the standard format used for version numbers,
# but would have to be set by the developer as a tag name.
# however, if the developer just sets a major and minor version with
# a tag, then git will add the number of commits since the tag. for example
#     0.1-10-devel
# indicates that this is the 10th commit on the devel branch after 0.1 patch.

set( TMP ${VERSION_FULL} )

# first number in version string will be the major version
# major number may be followed by a . or -
if( ${TMP} MATCHES "^v*([0-9]+)[\\.\\-]" )
  set( VERSION_MAJOR ${CMAKE_MATCH_1}  )
else()
  set( VERSION_MAJOR "X" )
endif()
# strip off the major number
string( REGEX REPLACE "^v*${VERSION_MAJOR}" "" TMP "${TMP}" )

# minor number will only be preceeded by a .
# but may be followed by a . or -
if( ${TMP} MATCHES "^[\\.]([0-9]+)[\\.\\-]" )
  set( VERSION_MINOR ${CMAKE_MATCH_1}  )
else()
  set( VERSION_MINOR "X" )
endif()
# strip off the minor number
string( REGEX REPLACE "^[\\.]${VERSION_MINOR}" "" TMP ${TMP} )

# patch may be preceeded by a . or -, but must be a number
if( ${TMP} MATCHES "^[\\.\\-]([0-9]+)" )
  set( VERSION_PATCH ${CMAKE_MATCH_1}  )
else()
  set( VERSION_PATCH "X" )
endif()

# build cmake-compatible version string
set(VERSION "${VERSION_MAJOR}")
if( NOT "${VERSION_MINOR}" STREQUAL "X" )
  set(VERSION "${VERSION}.${VERSION_MINOR}")
  if( NOT "${VERSION_PATCH}" STREQUAL "X" )
    set(VERSION "${VERSION}.${VERSION_PATCH}")
  endif()
else()
  if( NOT "${VERSION_PATCH}" STREQUAL "X" )
    set(VERSION "${VERSION}-${VERSION_PATCH}")
  endif()
endif()

# now set version information in the parent scope
set( ${VAR_PREFIX}_VERSION_FULL   ${VERSION_FULL}  PARENT_SCOPE )
set( ${VAR_PREFIX}_VERSION        ${VERSION}       PARENT_SCOPE )
set( ${VAR_PREFIX}_VERSION_MAJOR  ${VERSION_MAJOR} PARENT_SCOPE )
set( ${VAR_PREFIX}_VERSION_MINOR  ${VERSION_MINOR} PARENT_SCOPE )
set( ${VAR_PREFIX}_VERSION_PATCH  ${VERSION_PATCH} PARENT_SCOPE )

endfunction(GIT_VERSION)

