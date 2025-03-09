from setuptools import setup

package_name = 'voice_nav'

setup(
    name=package_name,
    version='0.0.2',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SeanChangX',
    maintainer_email='scx@gapp.nthu.edu.tw',
    description='Voice-controlled navigation for ROS 2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'speech_to_nav = voice_nav.speech_to_nav:main',
        ],
    },
)
