"""Preserve physical cargo ownership in the execution-state event."""
import re


def result_status(success, message):
    message = str(message or '')
    if success:
        return 'SUCCESS | cargo_on_carrier=true' if 'cargo_on_carrier=true' in message else 'SUCCESS'
    if re.search(r'\bcarrying_phase\s*=\s*true\b', message, re.IGNORECASE):
        return 'FAILED_WITH_LOAD | ' + message[:1000]
    return 'FAILURE' + (' | ' + message[:1000] if message else '')
