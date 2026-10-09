"""Ejecuta todas las celdas en una copia y conserva las salidas previas originales."""

import hashlib
import json
from pathlib import Path
import shutil
import tempfile

import nbformat
from nbconvert.preprocessors import ExecutePreprocessor


def verificar_notebook():
    # La copia permite ejecutar las celdas originales que regeneran señales.
    raiz = Path(__file__).resolve().parent
    protegidos = [p for p in raiz.rglob('*') if p.is_file() and
                  (p.suffix == '.wav' and 'pruebas' in p.parts or
                   p.name in ('configuraciones.json', 'metricas.csv') or
                   p.name.endswith('_caracterizacion.png'))]
    hashes = {p: hashlib.sha256(p.read_bytes()).hexdigest() for p in protegidos}
    original = nbformat.read(raiz / 'notebook.ipynb', as_version=4)
    with tempfile.TemporaryDirectory(prefix='verificacion_temporal_') as temporal:
        copia = Path(temporal) / 'tarea1_grupo'
        shutil.copytree(raiz, copia, ignore=shutil.ignore_patterns('__pycache__'))
        ejecutado = ExecutePreprocessor(timeout=600, kernel_name='python3').preprocess(
            original, {'metadata': {'path': str(copia)}})[0]
        # Se guardan solo los resultados nuevos; las celdas anteriores conservan sus salidas.
        notebook = json.loads((raiz / 'notebook.ipynb').read_text(encoding='utf-8'))
        for indice in range(21, len(notebook['cells'])):
            if notebook['cells'][indice]['cell_type'] == 'code':
                notebook['cells'][indice]['outputs'] = ejecutado.cells[indice].outputs
                notebook['cells'][indice]['execution_count'] = ejecutado.cells[indice].execution_count
        (raiz / 'notebook.ipynb').write_text(json.dumps(notebook, ensure_ascii=False, indent=1) + '\n', encoding='utf-8')
    # La comparación detecta cualquier modificación involuntaria al material original.
    assert all(hashlib.sha256(p.read_bytes()).hexdigest() == digest for p, digest in hashes.items())
    print(f'Notebook completo ejecutado: {len(original.cells)} celdas; {len(hashes)} archivos originales intactos.')


if __name__ == '__main__':
    # El directorio de trabajo no determina dónde se crean las salidas originales.
    verificar_notebook()
