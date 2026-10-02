"""Isolated module runtimes must not contaminate the existing ROS image."""
import sys
from pathlib import Path
import unittest
import yaml

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import gen_dockerfile_compose as generator


BASE=dict(_path='base',base_image={'amd64':'ubuntu:20.04'})
ISAAC=dict(_path='simulation/isaac-lab',deps={'pip':['forbidden-in-ros']},
    container=dict(service='isaac',arch=['amd64'],image='native:1',dockerfile='Dockerfile',gpu=True,
                   environment={'ISAAC_SCRIPT':'/work/example.py'},mounts=['cache:/root/.cache']))
OVERRIDE={'isaac':{'gpu_uuid':'GPU-1234-abcd'}}


class IsolatedContainerTest(unittest.TestCase):
    def test_dependencies_and_ros_environment_are_separated(self):
        dockerfile=generator.gen_dockerfile([BASE,ISAAC],'amd64',None,{})
        self.assertNotIn('forbidden-in-ros',dockerfile)
        document=yaml.safe_load(generator.gen_compose([BASE,ISAAC],'amd64','example',{},gpu=False,
            ros_master_port=11311,container_overrides=OVERRIDE))
        service=document['services']['isaac']
        self.assertEqual(service['runtime'],'nvidia')
        self.assertEqual(service['environment']['NVIDIA_VISIBLE_DEVICES'],'GPU-1234-abcd')
        self.assertNotIn('deploy',service)
        self.assertNotIn('ROS_MASTER_URI',service['environment'])
        self.assertEqual(service['build']['dockerfile'],'modules/simulation/isaac-lab/Dockerfile')
        self.assertEqual(service['healthcheck'],{'disable':True})
        self.assertIn('cache',document['volumes'])
        self.assertNotIn('runtime',document['services']['dev'])

    def test_old_stacks_have_only_dev(self):
        document=yaml.safe_load(generator.gen_compose([BASE],'amd64','old',{},gpu=False))
        self.assertEqual(list(document['services']),['dev'])
        self.assertNotIn('volumes',document)

    def test_uuid_environment_override(self):
        services=generator.isolated_services([ISAAC],'amd64','x',{'ISAAC_GPU_UUID':'GPU-abcd-9876'},
            {'isaac':{'gpu_uuid':'GPU-1234','gpu_uuid_env':'ISAAC_GPU_UUID'}})
        self.assertEqual(services['isaac']['environment']['NVIDIA_VISIBLE_DEVICES'],'GPU-abcd-9876')

    def test_reject_bad_uuid_unknown_override_and_name_collision(self):
        for overrides in ({'isaac':{'gpu_uuid':'0'}},{'typo':{}}, {'isaac':{'gpu_uuid':'GPU-abcd','image':'wrong'}}):
            with self.assertRaises(ValueError): generator.isolated_services([ISAAC],'amd64','x',{},overrides)
        with self.assertRaises(ValueError): generator.isolated_services([ISAAC,ISAAC],'amd64','x',{},OVERRIDE)


if __name__=='__main__': unittest.main()
