from setuptools import setup
from glob import glob
setup(name='dongfeng_autonomy',version='0.1.0',packages=['dongfeng_autonomy'],
      data_files=[('share/ament_index/resource_index/packages',['resource/dongfeng_autonomy']),
                  ('share/dongfeng_autonomy',['package.xml']),
                  ('share/dongfeng_autonomy/config',glob('config/*.json')+glob('config/*.npz'))],
      install_requires=['setuptools'],zip_safe=True,
      maintainer='Course experiment',maintainer_email='course@example.invalid',
      description='Autonomous perimeter driving',license='Proprietary',
      entry_points={'console_scripts':[
          'autonomy_node = dongfeng_autonomy.autonomy_node:main',
          'signal_node = dongfeng_autonomy.signal_node:main',
          'arbiter_node = dongfeng_autonomy.arbiter_node:main']})
