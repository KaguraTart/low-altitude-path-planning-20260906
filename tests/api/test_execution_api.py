"""Synchronous and job submissions preserve explicit candidate execution options."""
import time

import pytest
from fastapi.testclient import TestClient

from server.app import app
from server.v2 import example_request


def payload():
    value = example_request()
    task = value.pop('task')
    return dict(value, tasks=[dict(task, id=f'execution-{i}') for i in range(3)],
                execution={'workers': 2, 'batch_size': 2})


def test_sync_and_async_execution_reports():
    with TestClient(app) as client:
        response = client.post('/api/v2/plan/batch', json=payload())
        assert response.status_code == 200, response.text
        direct = response.json()
        submitted = client.post('/api/v2/jobs', json=payload())
        assert submitted.status_code == 202, submitted.text
        end = time.monotonic() + 10
        while time.monotonic() < end:
            job = client.get(submitted.json()['poll_url']).json()
            if job['status'] in {'succeeded', 'failed'}:
                break
            time.sleep(.01)
        assert job['status'] == 'succeeded', job
        asynchronous = job['result']
        assert direct['routes'] == asynchronous['routes']
        for value in (direct, asynchronous):
            report = value['execution_report']
            assert report['requested_workers'] == 2
            assert 1 <= report['actual_workers'] <= 2
            assert report['batch_size'] == 2
            assert report['batches'] == 2
            assert report['candidate_count'] == 3
            assert value['conflicts'] == []


@pytest.mark.parametrize('execution', [{'workers': True}, {'workers': 0}, {'workers': '2'}, {'batch_size': False}, {'batch_size': 0}, {'batch_size': '2'}])
@pytest.mark.parametrize('path', ['/api/v2/plan/batch', '/api/v2/jobs'])
def test_invalid_execution_is_rejected_before_submission(execution, path):
    with TestClient(app) as client:
        value = payload()
        value['execution'] = execution
        response = client.post(path, json=value)
        assert response.status_code == 400
        assert response.json()['error']['code'] == 'INVALID_INPUT'
