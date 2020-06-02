
var express = require('express');
var router = express.Router();

/* GET users listing. */
router.get('*', function(req, res, next) {
  res.redirect('/?route=' + encodeURIComponent(req.originalUrl));
});

module.exports = router;

