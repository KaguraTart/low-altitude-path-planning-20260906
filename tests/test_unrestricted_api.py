"""The HTTP contract accepts workload sizes without fixed resource quotas."""
from fastapi.testclient import TestClient

from server.app import app
from server.schemas import PlanBatchRequest, PlanSingleRequest
from server.v2 import BatchRequest, SingleRequest, ValidateRequest, example_request


def test_capabilities_report_no_service_resource_limits():
    result = TestClient(app).get('/api/v2/capabilities')
    assert result.status_code == 200
    body = result.json()
    assert body['max_registered_scenes'] is None
    assert body['max_registered_scene_bytes'] is None
    assert body['max_scene_radius_m'] is None
    assert all(value is None for value in body['scene_limits'].values())


def test_v1_and_v2_default_to_no_search_time_or_iteration_budget():
    example = example_request()
    assert SingleRequest.model_validate(example).time_budget_s is None
    batch = {**example, 'tasks': [example['task']]}
    batch.pop('task')
    assert BatchRequest.model_validate(batch).time_budget_s is None
    assert SingleRequest.model_validate({**example, 'time_budget_s': 301}).time_budget_s == 301
    assert PlanSingleRequest().max_iterations is None
    assert PlanBatchRequest(tasks=[]).max_iterations is None


def test_batch_above_previous_task_limit_is_accepted_by_request_model():
    example = example_request()
    task = SingleRequest.model_validate(example).task
    batch = BatchRequest(scene=example['scene'], tasks=[task] * 10001)
    assert len(batch.tasks) == 10001


def test_validation_above_previous_sample_limit_is_accepted_by_request_model():
    example = example_request()
    request = ValidateRequest(**example, trajectory_wkt=example['task']['start_wkt'],
                              vertex_times_s=[0.] * 100001)
    assert len(request.vertex_times_s) == 100001
