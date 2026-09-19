import importlib.util
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('stack_context',ROOT/'tools/stack_context.py')
context=importlib.util.module_from_spec(spec);spec.loader.exec_module(context)


class StackContextTest(unittest.TestCase):
    def test_shared_mavros_resolves_to_each_stack(self):
        for alias,name in context.ALIASES.items():
            r=context.resolve({'DSD_STACK':alias},module='control/mavros')
            self.assertEqual(r['container'],'drone-stack-'+name)
            self.assertIn('control/flight-safety',r['modules'])
            self.assertIn('odometry/optitrack',r['modules'])

    def test_cannot_silently_run_other_stacks_sensor(self):
        with self.assertRaisesRegex(ValueError,'not part'):
            context.resolve({'DSD_STACK':'aruco'},module='sensor/realsense-d435i')
        with self.assertRaisesRegex(ValueError,'not part'):
            context.resolve({'DSD_STACK':'risk-aware'},module='sensor/see3cam-24cug')

    def test_explicit_stack_and_container_conflict_fails(self):
        with self.assertRaisesRegex(ValueError,'conflicting'):
            context.resolve({'DSD_STACK':'aruco','DSD_CONTAINER':'drone-stack-d435i-voxblox'})

    def test_explicit_setup_argument_overrides_shell(self):
        r=context.resolve({'DSD_STACK':'risk-aware','DSD_CONTAINER':'drone-stack-d435i-voxblox'},explicit='aruco',module='control/mavros')
        self.assertEqual(r['stack'],'aruco-landing-jetson')

    def test_container_context_wins_over_saved_selection(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);(root/'config').mkdir()
            (root/'stacks').symlink_to(ROOT/'stacks');(root/'modules').symlink_to(ROOT/'modules')
            context.select('risk-aware',root)
            self.assertEqual(context.resolve({},root=root)['stack'],'d435i-voxblox')
            self.assertEqual(context.resolve({'DSD_STACK_NAME':'aruco-landing-jetson'},root=root)['stack'],'aruco-landing-jetson')
            self.assertEqual(context.resolve({'DSD_STACK':'aruco'},root=root)['stack'],'aruco-landing-jetson')

    def test_no_selection_requires_choice_and_invalid_name_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaisesRegex(ValueError,'select a stack'):
                context.resolve({},root=Path(tmp))
        for value in ['../../tmp','aruco; echo hi','missing-stack']:
            with self.assertRaises(ValueError):context.canonical(value)

    def test_gui_ports_do_not_collide(self):
        a=context.resolve({'DSD_STACK':'aruco'})['gui']
        b=context.resolve({'DSD_STACK':'risk-aware'})['gui']
        for key in ['display','vnc_port','web_port']:self.assertNotEqual(a[key],b[key])

    def test_shell_resolver_routes_without_docker_calls(self):
        env={k:v for k,v in os.environ.items() if not k.startswith('DSD_')}
        env['DSD_STACK']='aruco'
        p=subprocess.run(['bash','-c','source modules/select_stack.sh; dsd_select_stack control/mavros; echo "$DSD_CONTAINER"'],cwd=ROOT,env=env,text=True,capture_output=True)
        self.assertEqual(p.returncode,0,p.stderr)
        self.assertEqual(p.stdout.strip(),'drone-stack-aruco-landing-jetson')

    def test_real_control_wrappers_target_selected_container(self):
        # Intercept Docker at the process boundary; never start a real controller.
        with tempfile.TemporaryDirectory() as tmp:
            directory=Path(tmp);log=directory/'calls.jsonl'
            docker=directory/'docker'
            docker.write_text("#!/usr/bin/python3\nimport json,os,sys\nwith open(os.environ['TRACE_DOCKER'],'a') as f:f.write(json.dumps(sys.argv[1:])+'\\n')\nif sys.argv[1]=='inspect':print(os.environ['TEST_REPO']);sys.exit(0)\nif sys.argv[1]=='start':sys.exit(0)\nsys.exit(99)\n")
            docker.chmod(0o755)
            for alias,stack in context.ALIASES.items():
                for script in ['control_mavros.sh','control_flight-safety.sh']:
                    log.write_text('')
                    env={k:v for k,v in os.environ.items() if not k.startswith('DSD_')}
                    env.update(DSD_STACK=alias,TRACE_DOCKER=str(log),TEST_REPO=str(ROOT),PATH=str(directory)+':'+env['PATH'])
                    result=subprocess.run(['bash',str(ROOT/'scripts'/script)],env=env,text=True,capture_output=True)
                    import json
                    calls=[json.loads(line) for line in log.read_text().splitlines()]
                    self.assertTrue(calls,(result.stdout,result.stderr))
                    for call in calls:
                        targets=[arg for arg in call if arg.startswith('drone-stack-')]
                        self.assertEqual(targets,['drone-stack-'+stack],call)
                    self.assertEqual(result.returncode,99,result.stderr)

    def test_wrong_sensor_fails_before_touching_docker(self):
        env={k:v for k,v in os.environ.items() if not k.startswith('DSD_')}
        env['DSD_STACK']='aruco'
        result=subprocess.run(['bash',str(ROOT/'scripts/sensor_realsense-d435i.sh')],env=env,text=True,capture_output=True)
        self.assertEqual(result.returncode,2)
        self.assertIn('not part',result.stderr)


if __name__=='__main__':unittest.main()
