import matplotlib, matplotlib.pyplot as plt

'''
   Plotting
'''
my_title_spec = {'color'    : 'k', 'fontsize'   : 20 }

def prepare_fig(fig=None, window_title=None, figsize=(20.48, 10.24), margins=None):
    if fig is None:
        fig = plt.figure(figsize=figsize)
    else:
        plt.figure(fig.number)
    if margins is not None:
        left, bottom, right, top, wspace, hspace = margins
        fig.subplots_adjust(left=left, right=right, bottom=bottom, top=top,
                            hspace=hspace, wspace=wspace)
    #pdb.set_trace()
    if window_title is not None:
         fig.canvas.set_window_title(window_title)
    return fig
# def prepare_fig(fig=None, window_title=None, figsize=(20.48, 10.24), margins=None):
#     if fig == None:
#         fig = plt.figure(figsize=figsize)
#     #else:
#     #    plt.figure(fig.number)
#     if margins:
#         left, bottom, right, top, wspace, hspace = margins
#         fig.subplots_adjust(left=left, right=right, bottom=bottom, top=top,
#                             hspace=hspace, wspace=wspace)
#     if window_title:
#          fig.canvas.set_window_title(window_title)
#     return fig


def decorate(ax, title=None, xlab=None, ylab=None, legend=None, xlim=None, ylim=None, min_yspan=None):
    ax.xaxis.grid(color='k', linestyle='-', linewidth=0.2)
    ax.yaxis.grid(color='k', linestyle='-', linewidth=0.2)
    if xlab: ax.xaxis.set_label_text(xlab)
    if ylab: ax.yaxis.set_label_text(ylab)
    if title: ax.set_title(title, {'fontsize': 20 })
    if legend is not None:
        if legend == True: ax.legend(loc='best')
        else: ax.legend(legend, loc='best')
    if xlim is not None: ax.set_xlim(xlim[0], xlim[1])
    if ylim is not None: ax.set_ylim(ylim[0], ylim[1])
    if min_yspan is not None: ensure_yspan(ax, min_yspan)

def ensure_yspan(ax, yspan):
    ymin, ymax = ax.get_ylim()
    if ymax-ymin < yspan:
        ym =  (ymin+ymax)/2
        ax.set_ylim(ym-yspan/2, ym+yspan/2)

# def decorate(ax, title=None, xlab=None, ylab=None, legend=None, xlim=None, ylim=None):
#     ax.xaxis.grid(color='k', linestyle='-', linewidth=0.2)
#     ax.yaxis.grid(color='k', linestyle='-', linewidth=0.2)
#     if xlab:
#         ax.xaxis.set_label_text(xlab)
#     if ylab:
#         ax.yaxis.set_label_text(ylab)
#     if title:
#         ax.set_title(title, my_title_spec)
#     if legend <> None:
#         ax.legend(legend, loc='best')
#     if xlim <> None:
#         ax.set_xlim(xlim[0], xlim[1])
#     if ylim <> None:
#         ax.set_ylim(ylim[0], ylim[1])

def savefig(filename):
    if filename is not None:
        print('saving {}'.format(filename))
        plt.savefig(filename)
