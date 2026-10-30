"""Package project artifacts without machine-local environments and logs."""
from pathlib import Path
from zipfile import ZipFile, ZIP_DEFLATED
ROOT = Path(__file__).resolve().parents[1]
allowed_dirs = ['src', 'scripts', 'tests', 'gazebo', 'report', 'docs', 'results']
allowed_files = ['PROJECT_GUIDE.html', 'README.md', 'requirements.txt', 'requirements-lock.txt', 'MiniProject_FilterImplementation2.pdf']
files = [ROOT/f for f in allowed_files]
for folder in allowed_dirs:
    files.extend(p for p in (ROOT/folder).rglob('*') if p.is_file()
                 and not any(s in p.parts for s in ['__pycache__', 'ros_logs'])
                 and p.suffix not in ['.pyc', '.log'])
archive = ROOT/'EKF_SLAM_Submission.zip'
with ZipFile(archive, 'w', ZIP_DEFLATED) as z:
    for path in sorted(files):
        z.write(path, arcname=Path('EKF_SLAM_Submission')/path.relative_to(ROOT))
print(f'{archive}: {len(files)} files, {archive.stat().st_size/1024/1024:.2f} MiB')
