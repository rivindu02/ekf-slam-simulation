#!/usr/bin/env python3
"""Create a short presentation PDF using the measured experiment results."""
import json
from pathlib import Path
from reportlab.pdfgen import canvas
from reportlab.lib.colors import HexColor, white
from reportlab.lib.utils import ImageReader
from reportlab.pdfbase.pdfmetrics import stringWidth
ROOT = Path(__file__).resolve().parents[1]
s = json.loads((ROOT/'results/summary.json').read_text())['summary'][0]
slides = [
 ('EKF-Based 2D SLAM', ['Differential-drive robot with range–bearing landmarks',
                         'Jointly estimate robot pose and an initially unknown map',
                         'Student: ____________________'], None),
 ('Problem and scope', ['Odometry accumulates drift; the map is initially unknown.',
                        '10 stationary landmarks; known IDs; 360° sensing to 4.5 m.',
                        'Known starting pose; Gaussian simulated noise.',
                        'Python implementation plus Gazebo Classic / ROS 2 demo.'], None),
 ('Joint state and uncertainty', ['X = [x, y, heading, m1x, m1y, ..., mNx, mNy]',
                                  '23 states after all 10 landmarks are discovered.',
                                  'Full covariance preserves robot–landmark correlations.',
                                  'The filter receives no true landmark coordinates.'], None),
 ('Prediction and correction', ['Predict pose from left/right wheel travel; propagate covariance.',
                                'First sighting: inverse sensor model + correlated augmentation.',
                                'Repeat sighting: range/bearing innovation and Kalman gain.',
                                'Wrap angles; linear solves; Joseph covariance update.'], None),
 ('Trajectory and map', ['Black: truth; blue: EKF-SLAM; dashed orange: odometry.',
                         'Illustration uses seed 7, not the trial average.'], 'trajectory_map.png'),
 ('Paired Monte Carlo results', [f"20 runs per scenario: EKF RMSE {s['position_rmse_m_mean']:.3f} m; odometry {s['odometry_rmse_m_mean']:.3f} m.",
                                 'Error bars show sample standard deviation across runs.'], 'monte_carlo.png'),
 ('What happens during an outage?', ['No landmark corrections between 35 and 55 s.',
                                     'Robot position can recover while the map stays distorted.',
                                     'No-reuse ablation removes every repeated observation.'], 'revisit_dropout.png'),
 ('Accuracy is not confidence', ['Scale all assumed noise standard deviations by 0.35.',
                                 'Trajectory RMSE stays about 0.238 m.',
                                 'Mean NIS rises from 2.02 to 16.50: overconfident uncertainty.',
                                 'Mean NIS alone does not establish full SLAM consistency.'], None),
 ('Gazebo demonstration', ['Gazebo Classic 11 + ROS 2 Humble in devbox.',
                            'Encoder-mode odometry with added wheel noise.',
                            'Synthetic range/bearing from world poses; known IDs.',
                            'Completed headless physics run; GUI launcher supplied.',
                            'Physics-run metrics are separate from Monte Carlo results.'], None),
 ('Conclusions and limitations', ['Landmark reuse substantially reduces odometry drift.',
                                  'Outages and nonlinear errors can distort the map.',
                                  'Eleven automated tests and reproducible experiment outputs.',
                                  'Limitations: known IDs, ideal visibility, small map.',
                                  'Next: sensor extraction, robust association, more trajectories.'], None),
]
path = ROOT/'report/Presentation.pdf'
c = canvas.Canvas(str(path), pagesize=(960, 540))
notes = ['# Presentation outline', '']
for number, (title, bullets, figure) in enumerate(slides, 1):
    c.setFillColor(HexColor('#f4f7fb')); c.rect(0, 0, 960, 540, fill=1, stroke=0)
    c.setFillColor(HexColor('#173657')); c.rect(0, 462, 960, 78, fill=1, stroke=0)
    c.setFillColor(white); c.setFont('Helvetica-Bold', 26); c.drawString(35, 490, title)
    y = 430
    c.setFillColor(HexColor('#1d2b3a')); c.setFont('Helvetica', 16 if figure else 21)
    for bullet in bullets:
        words = ('- '+bullet.replace('–', '-').replace('°', ' deg')).split()
        line = ''
        size = 16 if figure else 21
        for word in words:
            candidate = (line+' '+word).strip()
            if stringWidth(candidate, 'Helvetica', size) > 875:
                c.drawString(40, y, line); y -= size+6; line = '  '+word
            else:
                line = candidate
        c.drawString(40, y, line)
        y -= 28 if figure else 48
    if figure:
        image = ImageReader(str(ROOT/'results'/figure)); w,h = image.getSize()
        height = min(315, y-40); width = min(800, height*w/h)
        height = width*h/w
        c.drawImage(image, (960-width)/2, 44, width, height, preserveAspectRatio=True)
    c.setFillColor(HexColor('#526579')); c.setFont('Helvetica', 10)
    c.drawString(35, 18, 'EKF-SLAM mini project | Simulated results')
    c.drawRightString(925, 18, str(number))
    c.showPage()
    notes.extend([f'## Slide {number}: {title}', *['- '+b for b in bullets], ''])
c.save()
(ROOT/'docs/PRESENTATION_OUTLINE.md').write_text('\n'.join(notes))
print(path)
