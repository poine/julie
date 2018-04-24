import numpy as np
import matplotlib, matplotlib.pyplot as plt, mpl_toolkits.mplot3d

def prepare_fig(fig, window_title, figsize=(20.48, 10.24), margins=None):
    if not fig:
        fig = plt.figure(figsize=figsize)
    else:
        plt.figure(fig.number)
    if margins:
        left, bottom, right, top, wspace, hspace = margins
        fig.subplots_adjust(left=left, right=right, bottom=bottom, top=top,
                            hspace=hspace, wspace=wspace)
    if window_title:
         fig.canvas.set_window_title(window_title)
    return fig


def set_axes_equal(ax=None):
    '''
    Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc..  This is one possible solution to Matplotlib's
    ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    '''
    if ax is None: ax = plt.gca()

    x_limits, y_limits, z_limits = ax.get_xlim3d(), ax.get_ylim3d(), ax.get_zlim3d()

    x_range, x_middle = abs(x_limits[1] - x_limits[0]), np.mean(x_limits)
    y_range, y_middle = abs(y_limits[1] - y_limits[0]), np.mean(y_limits)
    z_range, z_middle = abs(z_limits[1] - z_limits[0]), np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5*max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])


def decorate(ax, title=None, xlab=None, ylab=None, legend=None, xlim=None, ylim=None):
    ax.xaxis.grid(color='k', linestyle='-', linewidth=0.2)
    ax.yaxis.grid(color='k', linestyle='-', linewidth=0.2)
    if xlab:
        ax.xaxis.set_label_text(xlab)
    if ylab:
        ax.yaxis.set_label_text(ylab)
    if title:
        plt.title(title)#, my_title_spec)
    if legend <> None:
        plt.legend(legend, loc='best')
    if xlim <> None:
        ax.set_xlim(xlim[0], xlim[1])
    if ylim <> None:
        ax.set_ylim(ylim[0], ylim[1])
