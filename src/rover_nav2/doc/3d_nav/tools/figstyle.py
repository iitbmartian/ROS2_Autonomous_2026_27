"""Shared figure style for publication plots: light surface, recessive axes, the reference
palette (dataviz skill, references/palette.md). Categorical slots in fixed order, sequential is
one hue light to dark, diverging is blue to red through a gray midpoint."""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap

SURFACE = '#fcfcfb'
INK, INK2, MUTED = '#0b0b0b', '#52514e', '#898781'
GRID, AXIS = '#e1e0d9', '#c3c2b7'
# Fixed order. Configs keep their slot in every figure: 2D, 3D Ackermann, 3D swerve.
CATEGORICAL = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100', '#e87ba4', '#008300', '#6250d6', '#e34948']
CONFIG_COLOR = {'2d': CATEGORICAL[0], '3d': CATEGORICAL[1], 'swerve': CATEGORICAL[2]}
SEQ_BLUE = LinearSegmentedColormap.from_list(
    'seq_blue', ['#cde2fb', '#9ec5f4', '#6da7ec', '#3987e5', '#256abf', '#184f95', '#0d366b'])
DIVERGING = LinearSegmentedColormap.from_list(
    'div_blue_red', ['#184f95', '#6da7ec', '#f0efec', '#ec8a89', '#b52e2d'])
SEQ_BLUE.set_bad('#e1e0d9')
DIVERGING.set_bad('#e1e0d9')


def apply():
    plt.rcParams.update({
        'figure.facecolor': SURFACE, 'axes.facecolor': SURFACE, 'savefig.facecolor': SURFACE,
        'axes.edgecolor': AXIS, 'axes.labelcolor': INK2, 'axes.titlecolor': INK,
        'xtick.color': MUTED, 'ytick.color': MUTED, 'text.color': INK,
        'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': 0.6,
        'axes.spines.top': False, 'axes.spines.right': False,
        'font.size': 10, 'axes.titlesize': 11, 'axes.titleweight': 'semibold',
        'legend.frameon': False, 'lines.linewidth': 2.0, 'savefig.dpi': 200,
    })


def halo(width=2.5):
    """Surface-coloured outline so small labels stay readable over any terrain colour."""
    import matplotlib.patheffects as pe
    return [pe.withStroke(linewidth=width, foreground=SURFACE)]


def save(fig, path_no_ext):
    """PNG for slides and docs, SVG for papers."""
    fig.savefig(path_no_ext + '.png', bbox_inches='tight')
    fig.savefig(path_no_ext + '.svg', bbox_inches='tight')
    plt.close(fig)
