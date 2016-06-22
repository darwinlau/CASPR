% A wrapper class to replace runtests which is not supported on older
% matlab versions.
% Author: Jonathan EDEN
% Created: 2016
% Description: A wrapper class for unit testing.  Functionality should be
% the same as the runtest function that exists from MATLAB 2014a onwards.
function failed_tests = CASPR_tests(file_path)
    suite = matlab.unittest.TestSuite.fromFile(file_path);
    results = suite.run;
    failed_tests = [results.Failed];
end

