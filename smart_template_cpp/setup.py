from setuptools import setup
from glob import glob

package_name = 'smart_template_cpp'
setup(
    name=package_name,
    version='0.0.0',
    packages=['src'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),  
        ('share/' + package_name, ['plugin.xml']),      
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mariana Bernardes (BWH)',
    maintainer_email='mcostabernardesmatias@bwh.harvard.edu',
    description='SmartTemplate C++ package with GUI plugin',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'rqt_gui.plugin': [
            'smart_template_gui = src.smart_template_gui:SmartTemplateGUIPlugin',
        ],
    },
)