const webpack = require('webpack');
const pkg = require('./package.json');
const ESLintPlugin = require('eslint-webpack-plugin');

const libraryName = 'openteraWebrtcWebClient';

const banner = `${pkg.name}
${pkg.description}\n
@version v${pkg.version}
@author ${pkg.author}
@homepage ${pkg.homepage}
@repository ${pkg.repository.url}`;

const plugins = [
  new webpack.BannerPlugin(banner),
  new ESLintPlugin()
];

module.exports = (env, options) => {
  const libraryTarget = env['output-library-target'];
  const mode = options['mode'];

  return {
    entry: `${__dirname}/index.js`,
    devtool: 'source-map',
    output: {
      path: `${__dirname}/${libraryTarget === 'umd' ? 'dist' : 'lib'}`,
      filename: mode === 'development' ? `${libraryName}.js` : `${libraryName}.min.js`,
      library: libraryName,
      libraryTarget: libraryTarget || 'umd',
      globalObject: '(typeof self !== \'undefined\' ? self : this)', // TODO Hack (for Webpack 4+) to enable create UMD build which can be required by Node without throwing error for window being undefined (https://github.com/webpack/webpack/issues/6522)
      umdNamedDefine: true
    },
    module: {
      rules: [
        {
          test: /(\.jsx|\.js)$/,
          loader: 'babel-loader',
          exclude: /(node_modules|bower_components)/
        }
      ]
    },
    plugins: plugins
  };
};
