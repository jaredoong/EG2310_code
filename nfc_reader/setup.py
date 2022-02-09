from setuptools import setup

package_name = 'nfc_reader'
sub_packages = 'nfc_reader/pn532'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, sub_packages],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nfc_finder = nfc_reader.example_get_uid:main',
            'nfc_auto_nav = nfc_reader.nfc_autonav:main',
            'nfc_pub = nfc_reader.nfc_publisher:main',
            'nfc_sub = nfc_reader.nfc_subscriber:main',
            'nfc_auto_nav2 = nfc_reader.nfc_autonav2:main',
        ],
    },
)
