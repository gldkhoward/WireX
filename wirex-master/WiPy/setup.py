from distutils.core import setup, Extension
setup (name='WiPy', version = '1.0', 
       ext_modules=[Extension('WiPy',
                              include_dirs = ['/home/vfk/atlas'],
                              library_dirs = ['/home/vfk/atlas/lib'],
                              libraries = ['WireLib','tinyXML','WireLibPyBindings','motionPlanningLib'],
                              sources = ['WiPy.cpp','pyBindings.cpp'])
                   ]
      )
