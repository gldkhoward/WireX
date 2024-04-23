from distutils.core import setup, Extension
setup (name='WiPy', version = '1.0', 
       ext_modules=[Extension('WiPy',
                              include_dirs = ['/home/asp/ATLAS'],
                              library_dirs = ['/home/asp/ATLAS/lib'],
                              libraries = ['WireLib','tinyXML','WireLibPyBindings'],
                              sources = ['WiPy.cpp','pyBindings.cpp'])
                   ]
      )
