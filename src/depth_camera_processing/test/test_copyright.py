from ament_copyright.main import main
import pytest


@pytest.mark.skip(reason='Copyright headers not enforced for local development')
@pytest.mark.linter
@pytest.mark.copyright
def test_copyright():
    rc = main(argv=[])
    assert rc == 0, 'Found errors'
