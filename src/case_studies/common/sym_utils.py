# standard library
import inspect
from pathlib import Path
import re
import sys
from types import ModuleType

# 3rd-party
import sympy as sp
from sympy.physics.vector import dynamicsymbols as _dynamicsymbols
from sympy.physics.control.lti import TransferFunction as TF
from sympy.physics.vector.printing import vlatex
from IPython.display import Math, display
from IPython.core.getipython import get_ipython

_ipython = get_ipython()
_itype = _ipython.__class__.__name__
if _itype == "ZMQInteractiveShell":
    _is_notebook = True  # Jupyter notebook or qtconsole
elif _itype == "TerminalInteractiveShell":
    _is_notebook = False  # Terminal running IPython
else:
    _is_notebook = False

_printing_enabled = True


def is_notebook() -> bool:
    """
    Determines if running in a Jupyter environment.

    Returns:
        result (bool): True if running in a Jupyter environment, False otherwise.
    """
    return _is_notebook


def DynamicSymbol(name: str) -> sp.Function:
    """
    Wrapper around sympy.physics.vector.dynamicsymbols to make type checker
    happy when creating a single symbol.

    Args:
        name (str): name of the dynamicsymbol to create.
    Returns:
        result (sp.Function): the created dynamic symbol (a symbol that is a
            function of time).
    """
    result = _dynamicsymbols(name)
    assert isinstance(result, sp.Function)
    return result


def dynamicsymbols(names) -> list[sp.Function]:
    """
    Wrapper around sympy.physics.vector.dynamicsymbols to make type checker
    happy when creating multiple symbols.

    Args:
        names: names of the dynamicsymbol(s) to create. Works the same as input
            to sympy.symbols().

    Returns:
        result (list[sp.Function]): the created dynamic symbols (symbols that
            are function of time).
    """
    result = _dynamicsymbols(names)
    if not isinstance(result, list):
        raise TypeError(
            "Expected to create multiple symbols. "
            "Use DynamicSymbol() to create a single dynamic symbol."
        )
    symbols: list[sp.Function] = []
    for symbol in result:
        assert isinstance(symbol, sp.Function)
        symbols.append(symbol)
    return symbols


def TildeSymbol(name: str, deriv: int = 0) -> sp.Symbol:
    """
    Creates a sympy Symbol with a tilde over it for use in latex rendering. These
    can be used to represent linearized variables, but should not be used while
    deriving the full nonlinear equations of motion.

    Args:
        name (str): name of the symbol to create. name should not have "dot" or
            "ddot" in it; use the deriv argument instead.
        deriv (int): number of derivatives to indicate with dots above the tilde.
    Returns:
        result (sp.Symbol): the created symbol with a tilde over it.
    """
    name = name.replace("phi", "ϕ")
    name = name.replace("theta", "θ")
    name = name.replace("psi", "ψ")
    name = name.replace("tau", "τ")
    name = name.replace("Phi", "Φ")
    name = name.replace("Theta", "Θ")
    name = name.replace("Psi", "Ψ")
    name = name.replace("Tau", "T")

    tilde = "\u0303"
    dot = "\u0307"
    ddot = "\u0308"

    if deriv == 0:
        sym = sp.Symbol(f"{name}{tilde}")
    elif deriv == 1:
        sym = sp.Symbol(f"{name}{tilde}{dot}")
    elif deriv == 2:
        sym = sp.Symbol(f"{name}{tilde}{ddot}")
    else:
        raise ValueError("deriv must be 0, 1, or 2")
    return sym


def printsym(expr):
    """
    Renders latex from sympy expression when in a Jupyter-like environment.
    Terminals are limited to ACII text and can not render latex, as such, when
    calling this function from a terminal (or any non-Jupyter environment), it
    will use sympy's pretty_print function instead.

    Args:
        expr: sympy expression to display
    """
    if not _printing_enabled:
        return
    # msg = sep.join([vlatex(sp.Symbol(e)) if isinstance(e, str) else vlatex(e) for e in expr])
    if _is_notebook:
        display(Math(vlatex(expr)))
    else:
        sp.pretty_print(expr)


def printeq(lhs, rhs):
    """
    Renders latex from sympy expression when in a Jupyter-like environment.
    Terminals are limited to ACII text and can not render latex, as such, when
    calling this function from a terminal (or any non-Jupyter environment), it
    will print the latex string instead.

    Args:
        expr: sympy expression to display
    """
    if not _printing_enabled:
        return
    if isinstance(lhs, str):
        lhs = sp.Symbol(lhs)
    if isinstance(rhs, str):
        rhs = sp.Symbol(rhs)

    if _is_notebook:
        expr = " = ".join([vlatex(lhs), vlatex(rhs)])
        display(Math(expr))
    else:
        sp.pretty_print(sp.Eq(lhs, rhs, evaluate=False))
        print("")


def enable_printing(enable=True):
    """
    Enables or disables pretty printing of sympy expressions.

    Args:
        enable (bool): if True, enables pretty printing. If False, disables it.
    """
    global _printing_enabled
    _printing_enabled = enable


def MonicTF(tf: sp.Expr | TF) -> TF:
    """
    Converts a transfer function expression to monic form (leading coefficient
    of denominator is 1).

    Args:
        tf (sp.Expr | TF): transfer function expression (numerator/denominator).
        var (sp.Symbol): variable in the transfer function (usually s or z).
    Returns:
        tf_monic (TF): transfer function in monic form.
    Examples:
        `tf_monic = MonicTF(tf, s)`
    """
    s = sp.Symbol("s")
    if isinstance(tf, TF):
        num = tf.num
        den = tf.den
    else:
        num, den = sp.fraction(tf)
    leading_coeff = sp.LC(den, s)
    num_monic = num / leading_coeff
    den_monic = sp.cancel(den / leading_coeff)
    tf_monic = TF(num_monic, den_monic, s)
    return tf_monic


def calc_omega(R: sp.Matrix) -> sp.Matrix:
    """
    Calculates the angular velocity vector from a rotation matrix.

    Args:
        R (3x3 sympy matrix): a rotation matrix that is parameterized by
            dynamicsymbol variables that are functions of time.
    Returns:
        omega (3x1 sympy matrix): angular velocity vector
    Examples:
        `omega = calc_omega(R)`
    """
    t = sp.symbols("t")
    R_dot = R.diff(t)
    omega_skew_matrix = R_dot @ R.T
    omega = omega_skew_matrix.vee()
    return sp.Matrix(omega)


def find_coeffs(vector: sp.Matrix, q_dot: sp.Matrix) -> sp.Matrix:
    """
    Factors q_dot out of a velocity vector. The coefficients are the matrix that
    multiplies q_dot to yield the original velocity vector.

    Args:
        vector (3x1 sympy matrix):  linear/angular velocity vector
        q_dot (nx1 sympy matrix): time deriviative of the generalized coordinates

    Returns:
        coeffs (3xn sympy matrix): coefficient matrix such that
             `coeffs @ q_dot = vector`
    Examples:
        `Vi = find_coeffs(vi, qdot)`
        `Wi = find_coeffs(wi, qdot)`
    """
    coeffs = sp.zeros(3, len(q_dot))
    for i in range(len(vector)):
        for j in range(len(q_dot)):
            collected = sp.collect(sp.expand(vector[i]), q_dot[j])
            coeffs[i, j] = collected.coeff(q_dot[j])
    coeffs = sp.trigsimp(coeffs)
    return sp.Matrix(coeffs)


def rotx(angle: sp.Expr) -> sp.Matrix:
    """
    Returns a 3x3 matrix representing a rotation about the x-axis.

    Args:
        angle (sp.Expr): amount to rotate about x-axis (radians).
    Returns:
        R (sp.Matrix): a 3x3 rotation matrix.
    Examples:
        `R = rotx(theta)`
    """

    R = sp.Matrix(
        [
            [1, 0, 0],
            [0, sp.cos(angle), sp.sin(-angle)],
            [0, sp.sin(angle), sp.cos(angle)],
        ]
    )
    return R


def roty(angle: sp.Expr) -> sp.Matrix:
    """
    Returns a 3x3 matrix representing a rotation about the y-axis.

    Args:
        angle (sp.Expr): amount to rotate about y-axis (radians).
    Returns:
        R (sp.Matrix): a 3x3 rotation matrix.
    Examples:
        `R = roty(theta)`
    """

    R = sp.Matrix(
        [
            [sp.cos(angle), 0, sp.sin(angle)],
            [0, 1, 0],
            [sp.sin(-angle), 0, sp.cos(angle)],
        ]
    )
    return R


def rotz(angle: sp.Expr) -> sp.Matrix:
    """
    Returns a 3x3 matrix representing a rotation about the z-axis

    Args:
        angle (sp.Expr): amount to rotate about z-axis (radians).
    Returns:
        R (sp.Matrix): a 3x3 rotation matrix.
    Examples:
        `R = rotz(theta)`
    """

    R = sp.Matrix(
        [
            [sp.cos(angle), sp.sin(-angle), 0],
            [sp.sin(angle), sp.cos(angle), 0],
            [0, 0, 1],
        ]
    )
    return R


def write_eom_to_file(
    x: sp.Matrix,
    u: sp.Matrix,
    params: list[sp.Symbol],
    sys_module: ModuleType,
    remove_underscores: bool = False,
    filename="eom_generated.py",
    **labeled_expressions,
):
    """
    Converts symbolic expressions for EOM functions and writes them to a Python file.
    All of our EOM terms are functions of the state vector x and input vector u, as well
    as various system parameters. This function uses sympy's lambdify function to
    convert sympy expressions to Python functions, then writes those functions to a
    specified file.

    Args:
        x (sp.Matrix): the state vector with q as the first half and qdot as the second
            half (x = [q, qdot]). The way we have structured our code, these terms are
            expected to be sympy dynamic symbols (symbols that are functions of time).
        u (sp.Matrix): the input vector using regular sympy symbols (not dynamic).
        params (list[sp.Symbol]): list of sympy Symbols representing system parameters.
        sys_module (ModuleType): the module where the generated file will be saved
            (A_arm, B_pendulum, etc).
        remove_underscores (bool): if True, removes underscores from parameter names
            in the generated file. Keeping underscores matches the sympy symbols while
            removing them matches the parameter file variable names. This allows you
            to choose which style you prefer.
        filename: string representing the name of the file to generate.
        labeled_expressions: keyword arguments where the key is a string label
            representing the name of the expression (e.g., "qddot"), and the value is
            the sympy expression to be converted to Python code (the key and value
            may be the same depending on how you labeled your code).
    Note
    ------
    This function is designed specifically for the controlbook case studies and
    may not be suitable for general use without modification. If you wish to
    replicate similar functionality in your own projects, consider adapting
    this code to fit your specific needs. It has been commented to explain the
    various steps involved in the process.
    """
    # get name of script that is running
    main_file_path = Path(sys.modules["__main__"].__file__ or "")
    main_filename = main_file_path.name

    # determine where to save generated file
    save_dir = Path(sys_module.__file__ or "").parent
    save_file = save_dir / filename

    if save_file.exists():
        if is_notebook():
            print("Enter response in VSCode command bar at top of window.")
        overwrite = input(f"{filename} already exists...overwrite it? (y/n): ")
        if not overwrite.lower().startswith("y"):
            print("Not overwriting.")
            return
        else:
            print("Overwriting existing file.")

    # sympy doesn't know how to label dynamic symbols in generated code, so it
    # creates dummy variables instead. To avoid this, we convert all dynamic
    # symbols (functions of time) to regular sympy symbols before generating code.
    # Generated code will use the same labels used to create regular symbols.
    n = len(x) // 2
    q = x[:n]
    dynamic_subs = {}
    for sym in q:
        label = str(sym)
        if "(t)" in label:
            label = label.replace("(t)", "")
            dynamic_subs[sym] = sp.Symbol(label)
            symdot = sym.diff()
            dynamic_subs[symdot] = sp.Symbol(label + "dot")
    x_static = x.subs(dynamic_subs)

    # check that all inputs are sympy Symbols, not dynamic symbols
    for ui in u:
        if not isinstance(ui, sp.Symbol):
            raise TypeError("All inputs are expected to be sympy Symbols.")

    # Function to replace dummy variables with "x" and "u". Since the first two
    # arguments to all of our generated functions are x and u, which are sympy
    # Matrices that get replaced by dummy variables, this code replaces the
    # dummy variables with "x" and "u" in the generated function code.
    def replace_dummy_vars(expr) -> str:
        # Extract name of 1st and 2nd arguments of generated function
        match = re.search(r"def _lambdifygenerated\(\s*(\S+),\s*(\S+),", expr)
        if not match:
            return expr  # No match found, return original expression
        arg1, arg2 = match.group(1), match.group(2)

        # Define pattern to replace all occurrences of arg1 and arg2
        pattern = re.compile(r"\b(" + re.escape(arg1) + r"|" + re.escape(arg2) + r")\b")

        def replacement(m: re.Match):
            # arg1 should always be a dummy variable to be replaced with "x"
            if m.group(0) == arg1 and "_Dummy_" in arg1:
                return "x"
            # arg2 should be a dummy variable to be replaced with "u" if u is
            # used in the expression; otherwise, arg2 is a parameter and should
            # not be changed (which is why we check for "_Dummy_" in the name)
            elif m.group(0) == arg2 and "_Dummy_" in arg2:
                return "u"
            else:  # don't modify
                return m.group(0)

        result = pattern.sub(replacement, expr)
        return result

    # Potential improvement: check to see if current generated code is different
    # from previous version. If not different, skip overwriting file. Could also
    # check individual functions and only overwrite those that changed, rather
    # than the entire file. This would make it easier to add more functions in
    # the future without overwriting existing ones (if we want to generate code
    # for transfer functions and state space models later, for example).

    # opening file in "w" mode will overwrite existing file or create new one
    with open(save_file, "w") as f:
        # add file header
        f.write("#" * 80 + "\n")
        f.write(f"# This file was auto-generated by {main_filename}\n")
        f.write("#" * 80 + "\n")
        f.write("import numpy as np\n")

        # add all generated functions to file
        for label, expr in labeled_expressions.items():
            f.write("\n\n")

            # determine function arguments (could always pass in u, but u is not
            # always needed...this saves an unnecessary argument in some cases)
            original_str = str(expr)
            test_str = str(expr.subs({ui: 0 for ui in u}))
            if original_str == test_str:
                args = [x_static, *params]  # no u because expr does not use u
            else:
                args = [x_static, u, *params]

            # use lambdify to convert sympy expression to python function
            func = sp.lambdify(args, expr.subs(dynamic_subs), "numpy")

            # convert function object to python code string
            fn_str = inspect.getsource(func)

            # when using "numpy" as the module in lambdify, the generated code
            # uses "array", "sin", and "cos" without the "np." prefix, so we
            # need to add those prefixes back in. If other numpy functions are
            # used in the future, they will need to be added here as well.
            fn_str = re.sub("array", "np.array", fn_str)
            fn_str = re.sub("cos", "np.cos", fn_str)
            fn_str = re.sub("sin", "np.sin", fn_str)

            # sp.Matrix args are labeled as _Dummy_# (dummy variables), so we
            # replace those with x and u because those are the variables used
            # in equations of motion: xdot = f(x, u)...this code is not generic
            # and is specific to this use case. If you were generating code for
            # a different purpose, you would need another method to rename the
            # function arguments.
            fn_str = replace_dummy_vars(fn_str)

            # relabel generated function from sympy's default name
            fn_str = re.sub("_lambdifygenerated", f"calculate_{label}", fn_str)

            # these lines are only need if user passes in 2D arrays for x and u
            # use flatten() instead of squeeze() to avoid 0-d arrays
            fn_str = re.sub("= x", "= x.flatten()  # ensure 1D", fn_str)
            fn_str = re.sub("= u", "= u.flatten()  # ensure 1D", fn_str)

            # TODO: allow user to remove underscores or remove functionality?

            # This step is not strictly necessary. We used underscores when
            # creating the sympy symbols to make them print nicely with
            # subscripts in latex. However, our parameter files do not use
            # underscores, so we remove them here so that the generated code
            # matches the parameter file variable names.
            if remove_underscores:
                for p in params:
                    cur_label = str(p)
                    label = cur_label.replace("_", "")
                    if label == cur_label:
                        continue
                    fn_str = re.sub(rf"\b{cur_label}\b", label, fn_str)

            # label returned variable
            fn_str = re.sub("return", f"{label} =", fn_str)
            f.write(fn_str)

            # sympy forces vectors to be 2D; squeeze removes singular dimensions
            if f"{label} = np.array" in fn_str:
                f.write(f"    return {label}.squeeze()\n")
            else:
                f.write(f"    return {label}\n")

    print(f"Saved generated EOM to {save_file}")
