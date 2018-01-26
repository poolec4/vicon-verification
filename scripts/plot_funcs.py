import matplotlib.pyplot as plt

def plot_31_1(x, y, x_label='', y_label=['1', '2', '3'], title=''):
    fig, ax = plt.subplots(3,1)
    fig.suptitle(r'${{{}}}$'.format(title), fontsize=12)

    y_label_on = len(y_label)

    ax[0].plot(x[:], y[0][:], 'b')
    ax[0].set_ylabel(r'${{{}}}$'.format(y_label[0]))
    ax[0].grid('on')

    ax[1].plot(x[:], y[1][:], 'b')
    ax[1].set_ylabel(r'${{{}}}$'.format(y_label[1]))
    ax[1].grid('on')

    ax[2].plot(x[:], y[2][:], 'b')
    ax[2].set_ylabel(r'${{{}}}$'.format(y_label[2]))
    ax[2].grid('on')

    return


def plot_31_2(x1, y1, x2, y2, x_label='', y_label=['', '', ''], title='',
              legend=['', '']):
    fig, ax = plt.subplots(3,1)
    fig.suptitle(r'${{{}}}$'.format(title), fontsize=12)

    y_label_on = len(y_label)

    ax[0].plot(x1[:], y1[0][:], 'b', label=legend[0])
    ax[0].plot(x2[:], y2[0][:], 'g', label=legend[1])
    ax[0].set_ylabel(r'${{{}}}$'.format(y_label[0]))
    ax[0].set_xlabel(r'${{{}}}$'.format(x_label))
    ax[0].grid('on')
    if not legend[0] == '':
        ax[0].legend()

    ax[1].plot(x1[:], y1[1][:], 'b')
    ax[1].set_ylabel(r'${{{}}}$'.format(y_label[1]))
    ax[1].set_xlabel(r'${{{}}}$'.format(x_label))
    ax[1].plot(x2[:], y2[1][:], 'g')
    ax[1].grid('on')

    ax[2].plot(x1[:], y1[2][:], 'b')
    ax[2].set_ylabel(r'${{{}}}$'.format(y_label[2]))
    ax[2].set_xlabel(r'${{{}}}$'.format(x_label))
    ax[2].plot(x2[:], y2[2][:], 'g')
    ax[2].grid('on')

    return
