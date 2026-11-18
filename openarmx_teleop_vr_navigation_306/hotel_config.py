"""Resolve hotel evidence metadata from the editable VR task catalog.

The hotel recorder observes a selected task chain. It deliberately does not
interpret an action such as ``拿水`` or ``清理垃圾`` as a mandatory physical
operation: the operator decides what happens at the point and confirms it in
the headset. Point names only select evidence behavior; the route and posture
always come from ``task_actions.txt``.
"""

from copy import deepcopy
import hashlib
import json
from pathlib import Path

from .hotel_mission import validate_mission_config
from .simple_task_catalog import load_catalog, task_settings


def _point_kind(location):
    location = str(location).strip()
    if location.startswith('桌子'):
        return 'table'
    if location == '前台':
        return 'front_desk'
    if location == '垃圾桶':
        return 'trash_bin'
    return 'generic'


def _route_entry(task, original, index, used_ids=None):
    location = str(task.get('location', '')).strip()
    if not location:
        raise ValueError(f'任务串第{index}项缺少航点')
    base = deepcopy(original or {})
    area_id = str(base.get('id', '')).strip()
    if not area_id or (used_ids is not None and area_id in used_ids):
        # Repeating a waypoint in one chain must stay legal: keep the JSON
        # metadata (name/required/allowed_items) but mint a unique area id for
        # this occurrence instead of reusing ``round_table_1`` twice.
        digest = hashlib.sha256(f'{location}\0{index}'.encode()).hexdigest()[:10]
        area_id = f'route_{index:03d}_{digest}'
    base['id'] = area_id
    if used_ids is not None:
        used_ids.add(area_id)
    base.setdefault('name', location)
    base['waypoint'] = location
    base['task'] = str(task.get('task', '')).strip()
    if not base['task']:
        raise ValueError(f'任务串第{index}项缺少任务名称：{location}')
    base.update(task_settings(task))
    base['observation_kind'] = _point_kind(location)
    base['capture_evidence'] = base['observation_kind'] in {
        'table', 'front_desk', 'trash_bin'
    }
    return base


def hotel_catalog(config_path, task_catalog_path=None, *, task_catalog=None,
                  catalog_bytes=None):
    path = Path(config_path)
    content = path.read_bytes()
    payload = json.loads(content.decode('utf-8-sig'))
    source = Path(task_catalog_path) if task_catalog_path else path.parent / 'task_actions.txt'
    needs_txt = any(x.get('source_chain') for x in payload.get('missions', []))
    if needs_txt:
        if task_catalog is None:
            catalog = load_catalog(source)
        else:
            catalog = task_catalog
        if catalog_bytes is None:
            catalog_bytes = source.read_bytes()
    else:
        catalog = {}
        catalog_bytes = b''
    missions = []
    for original in payload.get('missions', []):
        mission = deepcopy(original)
        name = mission.get('source_chain')
        if name:
            chain = next((x for x in catalog.get('task_templates', [])
                          if x.get('name') == name), None)
            if chain is None:
                raise ValueError(f'任务文件缺少酒店任务串：{name}')
            old_by_waypoint = {
                str(x.get('waypoint', '')).strip(): x
                for x in (mission.get('areas') or [])
                if isinstance(x, dict)
            }
            used_ids = set()
            route = [
                _route_entry(task, old_by_waypoint.get(task.get('location')), index, used_ids)
                for index, task in enumerate(chain.get('tasks', []), 1)
            ]
            if not route:
                raise ValueError(f'酒店任务串不能为空：{name}')

            # Every row is an operator-confirmed point. No row is required to
            # mean pickup, cleaning, dumping, or docking.
            mission['generic_route'] = True
            mission['areas'] = route
            mission['route'] = deepcopy(route)
            mission['pickup'] = None
            mission['scheduled_final_disposal'] = False
            mission['start_waypoint'] = route[0]['waypoint']
            mission['return_waypoint'] = route[-1]['waypoint']

            bin_config = dict(mission.get('trash_bin') or {})
            bin_task = next((x for x in chain['tasks']
                             if str(x.get('location', '')).strip() == '垃圾桶'), None)
            if bin_task:
                bin_config.update({
                    'waypoint': bin_task['location'],
                    'task': bin_task.get('task', ''),
                    **task_settings(bin_task),
                })
            mission['trash_bin'] = bin_config
            mission['dock_pose'] = task_settings(
                next((x for x in reversed(chain['tasks'])
                      if str(x.get('location', '')).strip() == '充电桩'), {})
            )
            mission['catalog_source'] = {
                'path': str(source), 'chain': name,
                'sha256': hashlib.sha256(catalog_bytes).hexdigest(),
            }
        missions.append(validate_mission_config(mission))
    return {
        'record_root': str(payload.get('record_root', '~/Documents/sushi_task_records')),
        'missions': missions,
        'token': hashlib.sha256(content + b'\0' + catalog_bytes).hexdigest(),
        'recognition': deepcopy(payload.get('recognition') or {'provider': 'disabled'}),
    }
