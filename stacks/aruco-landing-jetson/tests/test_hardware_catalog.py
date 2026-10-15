import importlib.util
from pathlib import Path
import json
import sys

ROOT=Path(__file__).resolve().parents[3]
spec=importlib.util.spec_from_file_location('hardware_catalog',ROOT/'data/analysis/aruco/catalog_hardware_trials.py')
catalog=importlib.util.module_from_spec(spec);spec.loader.exec_module(catalog)


def record(session,index=0,outcome='success',pad='proposed1'):
    return dict(session=session,outcome=outcome,pad=dict(key=pad,layout_sha256='a'*64,configuration='layout.yaml'),
        set_sha256='b'*64,selection=dict(index=index,type='inside',reference_id=9595+index))


def test_only_latest_success_per_pad_path_is_visible():
    rows=[record('flight_1'),record('flight_2'),record('flight_3',outcome='failed'),record('flight_4',pad='baseline')]
    assert {r['session'] for r in catalog.selected_successes(rows)}=={'flight_2','flight_4'}


def test_catalog_reconstruction_preserves_originals_and_caches_scans(tmp_path,monkeypatch):
    import aruco_landing.campaign
    source=tmp_path/'source';output=tmp_path/'views';records={}
    for i,outcome in [(1,'success'),(2,'success'),(3,'failed')]:
        stem='flight_2026-10-15_00-00-0'+str(i);folder=source/stem;folder.mkdir(parents=True)
        bag=folder/(stem+'.bag');bag.write_bytes(b'original recording')
        manifest=bag.with_suffix('.runtime_manifest.yaml');manifest.write_text('finalized manifest')
        r=record(stem,outcome=outcome);r.update(bag_path=str(bag),manifest_path=str(manifest));records[bag]=r
    monkeypatch.setattr(aruco_landing.campaign,'flight_record',lambda p,owner:records[p])
    result=catalog.catalog_once(source,output,tmp_path,'2026-10-15')
    assert result['selected_successes']==1 and result['excluded_attempts']==2
    folders=list(output.glob('*/layout_*/set_*/*/ref_*/*'))
    assert len(folders)==1 and folders[0].name.endswith('02')
    assert (folders[0]/'flight.bag').is_symlink()
    assert all(b.read_bytes()==b'original recording' for b in records)
    monkeypatch.setattr(aruco_landing.campaign,'flight_record',lambda *a:(_ for _ in ()).throw(AssertionError('cache missed')))
    assert catalog.catalog_once(source,output,tmp_path,'2026-10-15')==result
    assert len(json.loads((output/'history.json').read_text())['excluded_from_active'])==2
