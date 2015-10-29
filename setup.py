from setuptools import setup

setup(name='Autopump',
      version='0.9.0',
      packages=['autopump'],
      entry_points={
          'console_scripts': [
              'autopump = autopump.__main__:main'
          ]
      },
      )