"""Apply explicit covariance overrides without altering sensor geometry."""
import re


def info_section(text, path):
    start, end = 0, len(text)
    for name in path:
        matches = list(re.finditer(r'(?m)^\s*' + re.escape(name) + r'\s*\{', text[start:end]))
        if len(matches) != 1:
            raise ValueError(f'Expected one INFO section {path}, found {len(matches)}')
        start += matches[0].end()
        depth, end = 1, start
        while depth and end < len(text):
            depth += (text[end] == '{') - (text[end] == '}')
            end += 1
        if depth:
            raise ValueError(f'Unclosed INFO section {path}')
        end -= 1
    return start, end


def apply_covariance_profile(config, override):
    for path in [('Init', 'Covariance'), ('Prediction', 'PredictionNoise')]:
        a, b = info_section(config, path)
        c, d = info_section(override, path)
        config = config[:a] + override[c:d] + config[b:]
    return config


def apply_image_gate(config, threshold):
    """Set the image innovation gate, retaining pose/zero-velocity gates."""
    config, count = re.subn(r'(?m)^(\s*MahalanobisTh\s+)[^;\n]+;',
                            lambda m: m[1] + str(threshold) + ';', config)
    if count != 1:
        raise ValueError(f'Expected one image MahalanobisTh, found {count}')
    return config
