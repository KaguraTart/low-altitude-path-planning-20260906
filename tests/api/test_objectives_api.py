"""Unrepresentable objective arithmetic is a typed API input error."""
import json

import pytest
from fastapi.testclient import TestClient

from server.app import app
from tests.planning.test_objectives import extreme_request


@pytest.mark.parametrize('case',['normalization','weighted_score','risk_integral','cost_distance','cost_time','energy'])
def test_extreme_finite_objective_arithmetic_does_not_return_http_500_or_nan(case):
    payload=json.loads(json.dumps(extreme_request(case),default=str))
    with TestClient(app) as client:
        response=client.post('/api/v2/plan/single',json=payload)
    assert response.status_code==400,response.text
    result=response.json()
    assert result['error']['code']=='INVALID_INPUT'
    assert 'nonfinite' in result['error']['message']
    assert 'NaN' not in response.text and 'Infinity' not in response.text
