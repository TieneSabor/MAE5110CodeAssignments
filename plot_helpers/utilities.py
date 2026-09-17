def param_get(param, key, default):
    """
    Get a parameter from a dictionary with a default value.
    """
    return param[key] if key in param else default