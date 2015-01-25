import warnings
from operator import itemgetter

import matplotlib.axes as maxes
from matplotlib.axes import Axes, rcParams
from matplotlib import cbook
import matplotlib.transforms as mtransforms
from matplotlib.transforms import Bbox
import matplotlib.collections as mcoll
from matplotlib import docstring
import matplotlib.scale as mscale
from matplotlib.tri.triangulation import Triangulation
import numpy as np
from matplotlib.colors import Normalize, colorConverter, LightSource
from mpl_toolkits.mplot3d import art3d

from matplotlib.tri import Triangulation, TriAnalyzer, UniformTriRefiner

def plot_trisurf_fixed(ax, *args, **kwargs):
        """
        ============= ================================================
        Argument      Description
        ============= ================================================
        *X*, *Y*, *Z* Data values as 1D arrays
        *color*       Color of the surface patches
        *cmap*        A colormap for the surface patches.
        *norm*        An instance of Normalize to map values to colors
        *vmin*        Minimum value to map
        *vmax*        Maximum value to map
        *shade*       Whether to shade the facecolors
        ============= ================================================

        The (optional) triangulation can be specified in one of two ways;
        either::

          plot_trisurf(triangulation, ...)

        where triangulation is a :class:`~matplotlib.tri.Triangulation`
        object, or::

          plot_trisurf(X, Y, ...)
          plot_trisurf(X, Y, triangles, ...)
          plot_trisurf(X, Y, triangles=triangles, ...)

        in which case a Triangulation object will be created.  See
        :class:`~matplotlib.tri.Triangulation` for a explanation of
        these possibilities.

        The remaining arguments are::

          plot_trisurf(..., Z)

        where *Z* is the array of values to contour, one per point
        in the triangulation.

        Other arguments are passed on to
        :class:`~mpl_toolkits.mplot3d.art3d.Poly3DCollection`

        **Examples:**

        .. plot:: mpl_examples/mplot3d/trisurf3d_demo.py
        .. plot:: mpl_examples/mplot3d/trisurf3d_demo2.py

        .. versionadded:: 1.2.0
            This plotting function was added for the v1.2.0 release.
        """

        had_data = ax.has_data()


        color = np.array(colorConverter.to_rgba(kwargs.pop('color', 'b')))

        cmap = kwargs.get('cmap', None)
        norm = kwargs.pop('norm', None)
        vmin = kwargs.pop('vmin', None)
        vmax = kwargs.pop('vmax', None)
        linewidth = kwargs.get('linewidth', None)
        shade = kwargs.pop('shade', cmap is None)
        lightsource = kwargs.pop('lightsource', None)

        tri, args, kwargs = Triangulation.get_from_args_and_kwargs(*args, **kwargs)
        z = np.asarray(args[0])

        ##mask according to distance
        ##################################
        ##################################
        def long_edges(x, y, triangles, minradio=0.3, maxradio=0.8):
            out = []
            for points in triangles:
                #print points
                a,b,c = points
                d0 = np.linalg.norm(x[a]-x[b])
                d1 = np.linalg.norm(x[c]-x[b])
                d2 = np.linalg.norm(x[c]-x[a])
                #d0 = np.sqrt( (x[a] - x[b]) **2 + (y[a] - y[b])**2 )
                #d1 = np.sqrt( (x[b] - x[c]) **2 + (y[b] - y[c])**2 )
                #d2 = np.sqrt( (x[c] - x[a]) **2 + (y[c] - y[a])**2 )
                max_edge = max([d0, d1, d2])
                #print points, max_edge
                if max_edge > maxradio or max_edge < minradio:
                    out.append(True)
                else:
                    out.append(False)

                d0 = np.linalg.norm(y[a]-y[b])
                d1 = np.linalg.norm(y[c]-y[b])
                d2 = np.linalg.norm(y[c]-y[a])
                max_edge = max([d0, d1, d2])
                if max_edge > maxradio or max_edge < minradio:
                    out[-1]=True
                else:
                    out[-1]=out[-1]
            return out

        triangles = tri.get_masked_triangles()
        xt = tri.x[triangles][...,np.newaxis]
        yt = tri.y[triangles][...,np.newaxis]

        #min_circle_ratio = 0.3
        #mask = TriAnalyzer(tri).get_flat_tri_mask(min_circle_ratio)
        #tri.set_mask(mask)

        mask = long_edges(xt,yt, tri.triangles)
        tri.set_mask(mask)

        ##################################
        ##################################

        triangles = tri.get_masked_triangles()
        xt = tri.x[triangles][...,np.newaxis]
        yt = tri.y[triangles][...,np.newaxis]
        zt = np.array(z)[triangles][...,np.newaxis]

        verts = np.concatenate((xt, yt, zt), axis=2)

        # Only need these vectors to shade if there is no cmap
        if cmap is None and shade:
            totpts = len(verts)
            v1 = np.empty((totpts, 3))
            v2 = np.empty((totpts, 3))
            # This indexes the vertex points
            which_pt = 0

        colset = []
        for i in xrange(len(verts)):
            avgzsum = verts[i,0,2] + verts[i,1,2] + verts[i,2,2]
            colset.append(avgzsum / 3.0)

            # Only need vectors to shade if no cmap
            if cmap is None and shade:
                v1[which_pt] = np.array(verts[i,0]) - np.array(verts[i,1])
                v2[which_pt] = np.array(verts[i,1]) - np.array(verts[i,2])
                which_pt += 1

        if cmap is None and shade:
            normals = np.cross(v1, v2)
        else:
            normals = []

        polyc = art3d.Poly3DCollection(verts, *args, **kwargs)

        if cmap:
            colset = np.array(colset)
            polyc.set_array(colset)
            if vmin is not None or vmax is not None:
                polyc.set_clim(vmin, vmax)
            if norm is not None:
                polyc.set_norm(norm)
        else:
            if shade:
                colset = ax._shade_colors(color, normals)
            else:
                colset = color
            polyc.set_facecolors(colset)

        ax.add_collection(polyc)
        ax.auto_scale_xyz(tri.x, tri.y, z, had_data)

        return polyc


