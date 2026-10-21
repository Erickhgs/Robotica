"""Compara a pose usada no controle com a verdade do Gazebo por tempo ROS."""
import argparse
from bisect import bisect_left
import csv
import json
import math
from pathlib import Path


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def interpolate_truth(times, points, t, max_gap=0.1):
    i = bisect_left(times, t)
    if i < len(times) and abs(times[i]-t) < 1e-9:
        return points[i]
    if i == 0 or i == len(times):
        return None
    dt = times[i]-times[i-1]
    if dt <= 0 or dt > max_gap:
        return None
    f = (t-times[i-1])/dt
    a, b = points[i-1], points[i]
    return (a[0]+f*(b[0]-a[0]), a[1]+f*(b[1]-a[1]),
            wrap(a[2]+f*wrap(b[2]-a[2])))


def summarize_trial(directory):
    directory = Path(directory)
    controls, truths = list(directory.glob('eight_*.csv')), list(directory.glob('ground_truth_*.csv'))
    if len(controls) != 1 or len(truths) != 1:
        raise ValueError(f'{directory}: esperado um CSV eight e um ground_truth por ensaio.')
    with controls[0].open(newline='') as stream:
        control = list(csv.DictReader(stream))
    with truths[0].open(newline='') as stream:
        ground = list(csv.DictReader(stream))
    times = [float(r['ros_time']) for r in ground]
    if any(b <= a for a, b in zip(times, times[1:])):
        raise ValueError(f'{directory}: relogio reiniciado ou duplicado; inicie um novo ensaio.')
    points = [tuple(float(r[k]) for k in ('x', 'y', 'yaw')) for r in ground]
    rows = []
    for row in control:
        if row['phase'] != 'run':
            continue
        truth = interpolate_truth(times, points, float(row['ros_time']))
        if truth is None:
            continue
        tx, ty, tyaw = truth
        r = {k: float(row[k]) for k in ('ros_time','time','x_ref','y_ref','yaw_ref',
                                       'x','y','yaw','v_command','w_command','saturated')}
        r.update(x_true=tx, y_true=ty, yaw_true=tyaw,
                 tracking_error_true=math.hypot(tx-r['x_ref'], ty-r['y_ref']),
                 estimation_error=math.hypot(tx-r['x'], ty-r['y']),
                 yaw_error_true=wrap(tyaw-r['yaw_ref']))
        rows.append(r)
    if len(rows) < 2:
        raise ValueError(f'{directory}: amostras sincronizadas insuficientes.')
    def rmse(key):
        return math.sqrt(sum(r[key]**2 for r in rows)/len(rows))
    def variation(key):
        return sum(abs(b[key]-a[key]) for a, b in zip(rows, rows[1:]))/(len(rows)-1)
    metadata_path = directory/'run_metadata.json'
    meta = json.loads(metadata_path.read_text()) if metadata_path.exists() else {}
    result = {
        'trial': str(directory), 'noise': meta.get('noise', ''),
        'seed': meta.get('seed', ''), 'omega': meta.get('omega', ''),
        'matched_samples': len(rows),
        'coverage_percent': 100*len(rows)/sum(r['phase']=='run' for r in control),
        'complete': any(r['phase']=='complete' for r in control),
        'rmse_tracking_true_m': rmse('tracking_error_true'),
        'rmse_estimation_m': rmse('estimation_error'),
        'rmse_yaw_true_rad': rmse('yaw_error_true'),
        'mean_abs_delta_v': variation('v_command'),
        'mean_abs_delta_w': variation('w_command'),
        'saturation_percent': 100*sum(r['saturated'] for r in rows)/len(rows),
    }
    with (directory/'comparison.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)
    return result


def main(args=None):
    parser = argparse.ArgumentParser(description='Avaliar pratica 04 com ground truth independente')
    parser.add_argument('directories', nargs='+', type=Path, help='Pastas de ensaios, ou raiz que as contem.')
    parser.add_argument('--output', type=Path, help='Tabela-resumo CSV para comparar os ensaios.')
    opts = parser.parse_args(args)
    directories = set()
    for path in opts.directories:
        directories.update(p.parent for p in path.rglob('ground_truth_*.csv'))
    if not directories:
        parser.error('Nenhum ensaio encontrado.')
    try:
        results = [summarize_trial(p) for p in sorted(directories)]
    except (OSError, ValueError, KeyError) as exc:
        parser.error(str(exc))
    for r in results:
        print(json.dumps(r, ensure_ascii=False, indent=2))
    if opts.output:
        opts.output.parent.mkdir(parents=True, exist_ok=True)
        with opts.output.open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(results[0]))
            writer.writeheader(); writer.writerows(results)


if __name__ == '__main__':
    main()
