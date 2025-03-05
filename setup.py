from setuptools import setup

package_name = 'smart_template_cpp'
setup(
    name=package_name,
    version='0.0.0',
    packages=['src'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['plugin.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='SmartTemplate C++ package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'rqt_gui.plugin': [
            'smart_template_gui = src.smart_template_gui:SmartTemplateGUIPlugin',
        ],
    },
)