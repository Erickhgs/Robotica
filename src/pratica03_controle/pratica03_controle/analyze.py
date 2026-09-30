"""Calcula RMSE e compara CSVs produzidos pelos nós, sem precisar do ROS."""
import argparse
import csv
import math
from pathlib import Path


def summarize(filename, phase='auto'):
    with Path(filename).expanduser().open(newline='', encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f'{filename}: arquivo sem amostras.')
    expected = {'phase', 'time', 'error_x', 'error_y', 'error_yaw', 'saturated', 'mode', 'kff', 'omega'}
    if not expected.issubset(rows[0]):
        raise ValueError(f'{filename}: não é um CSV de pratica03_controle.')
    selected_phases = {'run'} if any(r['phase'] == 'run' for r in rows) else {'active', 'hold'}
    if phase != 'auto':
        selected_phases = {phase}
    selected = [row for row in rows if row['phase'] in selected_phases]
    if not selected:
        raise ValueError(f'{filename}: nenhuma amostra da fase solicitada.')
    def rmse(key):
        return math.sqrt(sum(float(row[key])**2 for row in selected)/len(selected))
    first = selected[0]
    x, y = rmse('error_x'), rmse('error_y')
    return {
        'file': str(filename), 'mode': first['mode'], 'kff': float(first['kff']),
        'omega': float(first['omega']), 'samples': len(selected),
        'duration_s': float(selected[-1]['time'])-float(first['time']),
        'rmse_x_m': x, 'rmse_y_m': y, 'rmse_position_m': math.hypot(x, y),
        'rmse_yaw_rad': rmse('error_yaw'),
        'saturation_percent': 100.0*sum(int(r['saturated']) for r in selected)/len(selected),
        'trajectory_complete': any(r['phase'] == 'complete' for r in rows),
    }


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('files', nargs='+', help='Um ou mais CSVs de ensaios.')
    parser.add_argument('--phase', default='auto', choices=['auto', 'run', 'active', 'hold'])
    parser.add_argument('--output', help='Salvar a tabela de comparação em CSV.')
    parser.add_argument('--max-rmse', type=float, help='Critério em metros para busca experimental de Omega.')
    parser.add_argument('--max-yaw-rmse', type=float, default=0.2, help='Critério angular, rad; padrão 0.2.')
    opts = parser.parse_args(args)
    try:
        results = [summarize(path, opts.phase) for path in opts.files]
    except (OSError, ValueError, KeyError) as exc:
        parser.error(str(exc))
    for r in results:
        print(f"{r['file']}\n  modo={r['mode']} Kff={r['kff']:g} Omega={r['omega']:g} "
              f"n={r['samples']} duração={r['duration_s']:.2f}s\n"
              f"  RMSE: x={r['rmse_x_m']:.4f} m; y={r['rmse_y_m']:.4f} m; "
              f"posição={r['rmse_position_m']:.4f} m; yaw={r['rmse_yaw_rad']:.4f} rad\n"
              f"  saturação={r['saturation_percent']:.1f}% "
              f"trajetória_em_8_completa={r['trajectory_complete']}")
    if opts.output:
        dest = Path(opts.output).expanduser()
        dest.parent.mkdir(parents=True, exist_ok=True)
        with dest.open('w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(results[0]))
            writer.writeheader()
            writer.writerows(results)
        print(f'Comparação salva: {dest}')
    if opts.max_rmse is not None:
        if opts.max_rmse <= 0 or opts.max_yaw_rmse <= 0:
            parser.error('Os critérios de RMSE precisam ser positivos.')
        groups = {}
        for r in results:
            key = (r['mode'], r['kff'])
            groups.setdefault(key, [])
            if (r['trajectory_complete'] and r['rmse_position_m'] <= opts.max_rmse and
                    r['rmse_yaw_rad'] <= opts.max_yaw_rmse):
                groups[key].append(r['omega'])
        for (mode, kff), omegas in groups.items():
            result = f'{max(omegas):g} rad/s' if omegas else 'nenhum ensaio aprovado'
            print(f'Maior Omega TESTADO aprovado ({mode}, Kff={kff:g}): {result}')
        print('Resultado restrito aos ensaios fornecidos; não prova um máximo global.')


if __name__ == '__main__':
    main()
