find_package(unofficial-osg CONFIG REQUIRED)

set(OPENSCENEGRAPH_INCLUDE_DIR "C:/dev/vcpkg/installed/x64-windows/include")
set(OPENSCENEGRAPH_INCLUDE_DIRS "${OPENSCENEGRAPH_INCLUDE_DIR}")

set(OPENSCENEGRAPH_LIBRARIES
    unofficial::osg::osg
    unofficial::osg::osgDB
    unofficial::osg::osgGA
    unofficial::osg::osgUtil
    unofficial::osg::osgViewer
    unofficial::osg::osgText
    unofficial::osg::osgManipulator
    unofficial::osg::osgShadow
    unofficial::osg::osgSim
    unofficial::osg::OpenThreads
)

set(OSG_FOUND TRUE)
set(OSGDB_FOUND TRUE)
set(OSGGA_FOUND TRUE)
set(OSGUTIL_FOUND TRUE)
set(OSGVIEWER_FOUND TRUE)
set(OPENTHREADS_FOUND TRUE)
set(OSG_LIBRARY unofficial::osg::osg)
set(OSGDB_LIBRARY unofficial::osg::osgDB)
set(OSGGA_LIBRARY unofficial::osg::osgGA)
set(OSGUTIL_LIBRARY unofficial::osg::osgUtil)
set(OSGVIEWER_LIBRARY unofficial::osg::osgViewer)
set(OPENTHREADS_LIBRARY unofficial::osg::OpenThreads)
set(OSG_INCLUDE_DIR "${OPENSCENEGRAPH_INCLUDE_DIR}")
set(OSGDB_INCLUDE_DIR "${OPENSCENEGRAPH_INCLUDE_DIR}")
set(OSGGA_INCLUDE_DIR "${OPENSCENEGRAPH_INCLUDE_DIR}")
set(OSGUTIL_INCLUDE_DIR "${OPENSCENEGRAPH_INCLUDE_DIR}")
set(OSGVIEWER_INCLUDE_DIR "${OPENSCENEGRAPH_INCLUDE_DIR}")
set(OPENTHREADS_INCLUDE_DIR "${OPENSCENEGRAPH_INCLUDE_DIR}")
set(OpenSceneGraph_FOUND TRUE)
