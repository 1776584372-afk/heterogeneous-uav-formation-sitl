import math
from pathlib import Path
import tempfile
import unittest
from zipfile import ZipFile

from scripts.audit_acceleration_basis import docx_paragraphs, legacy_fits, step_segments, window_acceleration


class AccelerationBasisTests(unittest.TestCase):
    def test_repeated_phase_names_preserve_separate_acceleration_and_deceleration(self):
        rows=[]
        for k,(name,slope) in enumerate([('step_8',.5),('step_16',.5),('step_8',-.5)]):
            rows.extend(dict(phase=name,sim_s=k*20+j*.1,
                ref_speed=12+slope*j*.1,heading_forward=12+slope*j*.1) for j in range(101))
        self.assertEqual(len(step_segments(rows)),3)
        fits=legacy_fits(rows,'heading_forward')
        self.assertEqual([v['phase'] for v in fits],['step_8','step_16','step_8'])
        self.assertEqual([v['segment'] for v in fits],[0,1,2])
        for v,slope in zip(fits,[.5,.5,-.5]):
            self.assertAlmostEqual(v['measured_slope'],slope)
            self.assertEqual(v['max_speed_error'],0.)

    def test_small_speed_error_does_not_certify_instantaneous_acceleration(self):
        rows=[dict(phase='step_14',sim_s=j*.02,ref_speed=12+.2*j*.02,
                   airspeed=12+.2*j*.02+.04*math.sin(12*j*.02)) for j in range(601)]
        fit=legacy_fits(rows,'airspeed')[0]
        self.assertLess(fit['max_speed_error'],1.)
        self.assertAlmostEqual(fit['measured_slope'],.2,delta=.005)
        samples=[[r['sim_s'],r['airspeed'],0.] for r in rows]
        actual=window_acceleration(samples,[0,12],.2)
        self.assertGreater(actual['maximum'],.6)
        self.assertGreater(actual['upper_violation_samples'],0)

    def test_missing_clock_data_remains_unknown_and_duplicate_clocks_are_safe(self):
        self.assertIn('unavailable',window_acceleration([], [0,1], .4))
        samples=[[t,12+.3*t,0.] for t in [0,0,.1,.1,.2,.2]]
        result=window_acceleration(samples,[0,.2],.4)
        self.assertAlmostEqual(result['maximum'],.3)
        self.assertEqual(result['upper_violation_samples'],0)

    def test_docx_excerpts_preserve_equations_and_table_paragraph_order(self):
        xml='''<w:document xmlns:w="http://schemas.openxmlformats.org/wordprocessingml/2006/main"
          xmlns:m="http://schemas.openxmlformats.org/officeDocument/2006/math"><w:body>
          <w:p><w:r><w:t>limits</w:t></w:r><m:oMath><m:r><m:t>a=0.4</m:t></m:r></m:oMath></w:p>
          <w:tbl><w:tr><w:tc><w:p><w:r><w:t>unknown</w:t></w:r></w:p></w:tc></w:tr></w:tbl>
          </w:body></w:document>'''
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'paper.docx'
            with ZipFile(path,'w') as archive:
                archive.writestr('word/document.xml',xml)
            self.assertEqual(docx_paragraphs(path),['limitsa=0.4','unknown'])


if __name__=='__main__':
    unittest.main()
