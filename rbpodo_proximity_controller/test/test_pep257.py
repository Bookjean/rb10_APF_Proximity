import pytest
from ament_pep257.main import main


@pytest.mark.linter
def test_pep257():
    rc = main(argv=['--add-ignore', 'D100,D101,D102,D103,D104,D105,D106,D107'])
    assert rc == 0, 'Found code style errors / warnings'
